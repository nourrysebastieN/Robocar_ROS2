// robocar_teleop : manette F710 lue directement dans /dev/input (libevdev), sans SDL ni /joy.
// Logique de conduite reprise de le_loto/RoboCar/src/control/speed.c (soukoukou), en fraction
// de max_speed au lieu d'unités de duty VESC. Conception : README.md de ce paquet.
#include <fcntl.h>
#include <glob.h>
#include <libevdev/libevdev.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robocar_msgs/msg/teleop_state.hpp"
#include "std_msgs/msg/bool.hpp"

using ackermann_msgs::msg::AckermannDriveStamped;
using robocar_msgs::msg::TeleopState;
using std_msgs::msg::Bool;

namespace
{

// Premier F710 trouvé, quel que soit le numéro de série du dongle
std::string find_f710()
{
  glob_t g;
  std::string path;

  if (glob("/dev/input/by-id/*F710*-event-joystick", 0, nullptr, &g) == 0 && g.gl_pathc > 0) {
    path = g.gl_pathv[0];
  }
  globfree(&g);
  return path;
}

// Valeur d'un axe ramenée à [lo, hi] d'après les bornes annoncées par le pilote
double normalize(const libevdev * dev, unsigned int code, double lo, double hi)
{
  const input_absinfo * info = libevdev_get_abs_info(dev, code);
  if (info == nullptr || info->maximum == info->minimum) {
    return 0.0;
  }
  const double t = (static_cast<double>(info->value) - info->minimum) /
    (static_cast<double>(info->maximum) - info->minimum);
  return std::clamp(lo + t * (hi - lo), lo, hi);
}

double step_toward(double value, double target, double step)
{
  if (std::abs(target - value) <= step) {
    return target;
  }
  return value + (target > value ? step : -step);
}

}  // namespace

class RobocarTeleop : public rclcpp::Node
{
public:
  RobocarTeleop()
  : Node("robocar_teleop")
  {
    device_ = declare_parameter<std::string>("device", "");
    reconnect_period_ = declare_parameter("reconnect_period", 2.0);
    const double publish_rate = declare_parameter("publish_rate", 50.0);
    const double ramp_rate = declare_parameter("ramp_rate", 20.0);
    const double state_rate = declare_parameter("state_rate", 20.0);
    max_speed_ = declare_parameter("max_speed", 1.0);
    max_steering_angle_ = declare_parameter("max_steering_angle", 0.34);
    stick_deadzone_ = declare_parameter("stick_deadzone", 0.05);
    throttle_curve_ = declare_parameter("throttle_curve", 2.0);
    accel_step_ = declare_parameter("accel_step", 0.029);
    decel_step_ = declare_parameter("decel_step", 0.071);
    brake_step_ = declare_parameter("brake_step", 0.286);
    cross_step_ = declare_parameter("cross_step", 0.357);
    cross_hold_ticks_ = declare_parameter("cross_hold_ticks", 4);
    if (publish_rate <= 0.0 || ramp_rate <= 0.0 || state_rate <= 0.0) {
      throw std::invalid_argument("publish_rate, ramp_rate et state_rate doivent être > 0");
    }

    teleop_pub_ = create_publisher<AckermannDriveStamped>("teleop", 10);
    enable_pub_ = create_publisher<Bool>("drive_enable", 10);
    state_pub_ = create_publisher<TeleopState>("teleop/state", 10);

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this]() {publish_tick();});
    ramp_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / ramp_rate), [this]() {ramp_tick();});
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / state_rate), [this]() {state_tick();});
    reconnect_timer_ = create_wall_timer(
      std::chrono::duration<double>(reconnect_period_), [this]() {connect();});
    connect();
  }

  ~RobocarTeleop() override {disconnect();}

private:
  // ---------------------------------------------------------------- manette

  bool connected() const {return dev_ != nullptr;}

  void connect()
  {
    if (connected()) {
      return;
    }
    const std::string path = device_.empty() ? find_f710() : device_;
    if (path.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "aucune F710 dans /dev/input/by-id, nouvel essai");
      return;
    }
    fd_ = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000, "ouverture de %s impossible : %s",
        path.c_str(), std::strerror(errno));
      return;
    }
    const int rc = libevdev_new_from_fd(fd_, &dev_);
    if (rc < 0) {
      RCLCPP_ERROR(get_logger(), "libevdev sur %s : %s", path.c_str(), std::strerror(-rc));
      close(fd_);
      fd_ = -1;
      dev_ = nullptr;
      return;
    }
    RCLCPP_INFO(get_logger(), "manette \"%s\" sur %s", libevdev_get_name(dev_), path.c_str());
    reset_drive();
  }

  void disconnect()
  {
    if (dev_ != nullptr) {
      libevdev_free(dev_);
      dev_ = nullptr;
    }
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    reset_drive();
  }

  // Vide la file d'événements. libevdev garde l'état courant de chaque bouton et axe.
  void read_events()
  {
    input_event ev;
    unsigned int flag = LIBEVDEV_READ_FLAG_NORMAL;

    for (;;) {
      const int rc = libevdev_next_event(dev_, flag, &ev);
      if (rc == -EAGAIN) {
        if (flag == LIBEVDEV_READ_FLAG_SYNC) {
          flag = LIBEVDEV_READ_FLAG_NORMAL;  // fin de la resynchronisation
          continue;
        }
        return;
      }
      if (rc == LIBEVDEV_READ_STATUS_SYNC) {
        flag = LIBEVDEV_READ_FLAG_SYNC;  // événements perdus : libevdev rattrape l'état
        continue;
      }
      if (rc < 0) {
        RCLCPP_ERROR(get_logger(), "manette perdue (%s) : plus rien n'est publié", std::strerror(-rc));
        disconnect();
        return;
      }
      // Bascule avant / arrière sur l'appui seulement (R1 dans soukoukou, Y ici)
      if (ev.type == EV_KEY && ev.code == BTN_WEST && ev.value == 1) {
        reverse_ = !reverse_;
        target_ = 0.0;
      }
      if (ev.type == EV_KEY && ev.code == BTN_MODE && ev.value == 1) {
        reset_drive();
      }
    }
  }

  bool pressed(unsigned int code) const
  {
    return connected() && libevdev_get_event_value(dev_, EV_KEY, code) != 0;
  }

  double steer() const
  {
    const double x = normalize(dev_, ABS_X, -1.0, 1.0);
    return std::abs(x) < stick_deadzone_ ? 0.0 : x;
  }

  // ---------------------------------------------------------------- conduite

  void reset_drive()
  {
    speed_ = 0.0;
    target_ = 0.0;
    braking_ = false;
    cross_hold_ = 0;
  }

  // update_speed() de soukoukou : rampe lente en accélération, rapide en freinage,
  // passage par zéro avec une pause au point mort
  void ramp_tick()
  {
    if (!connected()) {
      return;
    }
    read_events();
    if (!connected()) {
      return;
    }
    // Sans LB, la rampe reste à 0 : un nouvel appui ne repart pas de l'ancienne vitesse
    if (!pressed(BTN_TL)) {
      reset_drive();
      return;
    }

    const double gas = normalize(dev_, ABS_RZ, 0.0, 1.0);
    const double brake = normalize(dev_, ABS_Z, 0.0, 1.0);
    if (brake > 0.0) {
      braking_ = true;
      target_ = 0.0;
      const double step = brake_step_ * (1.0 + 2.0 * brake);
      speed_ = step_toward(speed_, target_, step);
      return;
    }
    if (braking_) {
      braking_ = false;
      speed_ = 0.0;  // relâcher le frein : retour au point mort, comme soukoukou
    }
    target_ = gas > 0.0 ? std::pow(gas, throttle_curve_) * (reverse_ ? -1.0 : 1.0) : 0.0;

    if (cross_hold_ > 0) {
      cross_hold_--;
      speed_ = 0.0;
      return;
    }
    if (speed_ == target_) {
      return;
    }
    if ((speed_ > 0.0 && target_ < 0.0) || (speed_ < 0.0 && target_ > 0.0)) {
      if (std::abs(speed_) <= cross_step_) {
        speed_ = 0.0;
        cross_hold_ = cross_hold_ticks_;
      } else {
        speed_ += speed_ > 0.0 ? -cross_step_ : cross_step_;
      }
      return;
    }
    const double step = std::abs(target_) > std::abs(speed_) ? accel_step_ : decel_step_;
    speed_ = step_toward(speed_, target_, step);
  }

  // ---------------------------------------------------------------- publication

  // Rien n'est publié sans manette : le mux masque alors /drive, puis le watchdog arrête
  void publish_tick()
  {
    if (!connected()) {
      return;
    }
    read_events();
    if (!connected()) {
      return;
    }
    AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    if (pressed(BTN_TL)) {
      cmd.drive.speed = static_cast<float>(speed_ * max_speed_);
      // REP-103 : angle positif = gauche ; stick à gauche = X négatif
      cmd.drive.steering_angle = static_cast<float>(-steer() * max_steering_angle_);
    }
    teleop_pub_->publish(cmd);

    // false = /drive déverrouillé, publié en continu tant que RB est maintenu
    if (pressed(BTN_TR)) {
      Bool unlock;
      unlock.data = false;
      enable_pub_->publish(unlock);
    }
  }

  void state_tick()
  {
    if (!connected()) {
      return;
    }
    TeleopState s;
    s.header.stamp = now();
    s.throttle = static_cast<float>(
      std::clamp(normalize(dev_, ABS_RZ, 0.0, 1.0) - normalize(dev_, ABS_Z, 0.0, 1.0), -1.0, 1.0));
    s.steer = static_cast<float>(normalize(dev_, ABS_X, -1.0, 1.0));
    s.direction = reverse_ ? TeleopState::DIRECTION_REVERSE : TeleopState::DIRECTION_FORWARD;
    s.braking = braking_;
    s.target_fraction = static_cast<float>(std::abs(target_));
    s.fraction = static_cast<float>(std::abs(speed_));
    s.manual_enabled = pressed(BTN_TL);
    s.drive_enabled = pressed(BTN_TR);
    state_pub_->publish(s);
  }

  std::string device_;
  double reconnect_period_;
  double max_speed_, max_steering_angle_, stick_deadzone_, throttle_curve_;
  double accel_step_, decel_step_, brake_step_, cross_step_;
  int64_t cross_hold_ticks_;

  int fd_ = -1;
  libevdev * dev_ = nullptr;

  // Vitesse et cible en fraction signée de max_speed (+ = avant)
  double speed_ = 0.0;
  double target_ = 0.0;
  bool reverse_ = false;
  bool braking_ = false;
  int64_t cross_hold_ = 0;

  rclcpp::Publisher<AckermannDriveStamped>::SharedPtr teleop_pub_;
  rclcpp::Publisher<Bool>::SharedPtr enable_pub_;
  rclcpp::Publisher<TeleopState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_, ramp_timer_, state_timer_, reconnect_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobocarTeleop>());
  rclcpp::shutdown();
  return 0;
}
