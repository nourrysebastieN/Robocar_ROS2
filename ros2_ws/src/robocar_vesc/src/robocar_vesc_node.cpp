// robocar_vesc : /ackermann_cmd -> VESC par l'UART de la Jetson, avec la bibliothèque wesc.
// Conception : README.md de ce paquet et docs/WESC_JAZZY.md §3 et §6.
//
// Un seul fil (timer à telemetry_rate) écrit sur le port : consigne, ALIVE, GET_VALUES.
// Le callback ackermann_cmd ne fait que mémoriser la consigne et son heure.
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "wesc/wesc.h"

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class RobocarVesc : public rclcpp::Node
{
public:
  RobocarVesc()
  : Node("robocar_vesc"), steady_(RCL_STEADY_TIME)
  {
    port_ = declare_parameter<std::string>("port", "/dev/vesc");
    baud_ = declare_parameter("baud", 115200);
    const std::string mode = declare_parameter<std::string>("motor_mode", "speed");
    if (mode != "speed" && mode != "duty") {
      throw std::invalid_argument("motor_mode doit valoir 'speed' ou 'duty', pas '" + mode + "'");
    }
    duty_mode_ = mode == "duty";
    erpm_gain_ = declare_parameter("speed_to_erpm_gain", 4614.0);
    erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
    speed_min_ = declare_parameter("speed_min", -5000.0);
    speed_max_ = declare_parameter("speed_max", 5000.0);
    max_speed_ = declare_parameter("max_speed", 1.0);
    duty_max_ = declare_parameter("duty_max", 0.20);
    duty_deadband_ = declare_parameter("duty_deadband", 0.06);
    invert_motor_ = declare_parameter("invert_motor", false);
    servo_gain_ = declare_parameter("steering_angle_to_servo_gain", -0.88);
    servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.50);
    servo_min_ = declare_parameter("servo_min", 0.20);
    servo_max_ = declare_parameter("servo_max", 0.80);
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.1);
    const double rate = declare_parameter("telemetry_rate", 50.0);
    read_timeout_ms_ = declare_parameter("read_timeout_ms", 20);
    if (max_speed_ <= 0.0 || rate <= 0.0 || cmd_timeout_ <= 0.0) {
      throw std::invalid_argument("max_speed, telemetry_rate et cmd_timeout doivent être > 0");
    }

    state_pub_ = create_publisher<VescStateStamped>("sensors/core", 10);
    servo_pub_ = create_publisher<Float64>("sensors/servo_position_command", 10);
    cmd_sub_ = create_subscription<AckermannDriveStamped>(
      "ackermann_cmd", rclcpp::QoS(10),
      [this](AckermannDriveStamped::ConstSharedPtr msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = steady_.now();
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "port %s à %ld bauds, mode %s, watchdog %.2f s",
      port_.c_str(), baud_, mode.c_str(), cmd_timeout_);
  }

  ~RobocarVesc() override
  {
    if (fd_ >= 0) {
      wesc_set_duty(fd_, 0.0f);
      wesc_set_servo(fd_, static_cast<float>(servo_offset_));
      wesc_close(fd_);
    }
  }

private:
  bool ensure_open()
  {
    if (fd_ >= 0) {
      return true;
    }
    fd_ = wesc_open(port_.c_str(), static_cast<int>(baud_));
    if (fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "ouverture de %s impossible : %s",
        port_.c_str(), std::strerror(errno));
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s ouvert", port_.c_str());
    last_servo_.reset();
    return true;
  }

  void serial_error(const char * what)
  {
    RCLCPP_ERROR(get_logger(), "%s : %s, réouverture au prochain tick", what, std::strerror(errno));
    wesc_set_duty(fd_, 0.0f);
    wesc_close(fd_);
    fd_ = -1;
  }

  // Envoie la commande moteur ; vitesse nulle = duty 0 (moteur libre, pas de maintien en RPM)
  int send_motor(double speed)
  {
    if (speed == 0.0 || !std::isfinite(speed)) {
      return wesc_set_duty(fd_, 0.0f);
    }
    if (!duty_mode_) {
      const double erpm = std::clamp(erpm_gain_ * speed + erpm_offset_, speed_min_, speed_max_);
      return wesc_set_rpm(fd_, static_cast<int32_t>(std::lround(erpm)));
    }
    // Mode duty : saut de la zone morte du moteur, comme THROTTLE_DEADBAND dans soukoukou
    const double f = std::min(std::abs(speed) / max_speed_, 1.0);
    double duty = duty_deadband_ + f * (duty_max_ - duty_deadband_);
    if ((speed < 0.0) != invert_motor_) {
      duty = -duty;
    }
    return wesc_set_duty(fd_, static_cast<float>(duty));
  }

  void tick()
  {
    if (!ensure_open()) {
      return;
    }

    const bool fresh = last_cmd_time_ &&
      (steady_.now() - *last_cmd_time_).seconds() < cmd_timeout_;
    if (fresh) {
      if (send_motor(last_cmd_->drive.speed) == -1) {
        return serial_error("écriture moteur");
      }
      const double servo = std::clamp(
        servo_gain_ * last_cmd_->drive.steering_angle + servo_offset_, servo_min_, servo_max_);
      if (!last_servo_ || *last_servo_ != servo) {
        if (wesc_set_servo(fd_, static_cast<float>(servo)) == -1) {
          return serial_error("écriture servo");
        }
        last_servo_ = servo;
      }
      Float64 servo_msg;
      servo_msg.data = servo;
      servo_pub_->publish(servo_msg);
      // ALIVE seulement avec une consigne valide : sinon le timeout du VESC (500 ms) coupe
      if (wesc_send_alive(fd_) == -1) {
        return serial_error("écriture ALIVE");
      }
    } else {
      if (wesc_set_duty(fd_, 0.0f) == -1) {
        return serial_error("écriture moteur");
      }
      if (last_cmd_time_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "rien sur ackermann_cmd depuis %.2f s : moteur à 0",
          cmd_timeout_);
      }
    }

    if (wesc_request_values(fd_) == -1) {
      return serial_error("écriture GET_VALUES");
    }
    wesc_values_t v;
    if (wesc_read_values(fd_, &v, static_cast<int>(read_timeout_ms_)) == -1) {
      if (errno == ETIMEDOUT || errno == EBADMSG || errno == EPROTO || errno == EMSGSIZE) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "réponse GET_VALUES : %s", std::strerror(errno));
        return;
      }
      return serial_error("lecture GET_VALUES");
    }
    publish_state(v);
  }

  void publish_state(const wesc_values_t & v)
  {
    VescStateStamped msg;
    msg.header.stamp = now();
    auto & s = msg.state;
    s.temp_fet = v.temp_fet;
    s.temp_motor = v.temp_motor;
    s.current_motor = v.avg_motor_current;
    s.current_input = v.avg_input_current;
    s.avg_id = v.avg_id;
    s.avg_iq = v.avg_iq;
    s.duty_cycle = v.duty_cycle_now;
    s.speed = v.rpm;
    s.voltage_input = v.v_in;
    s.charge_drawn = v.amp_hours;
    s.charge_regen = v.amp_hours_charged;
    s.energy_drawn = v.watt_hours;
    s.energy_regen = v.watt_hours_charged;
    s.displacement = v.tachometer;
    s.distance_traveled = v.tachometer_abs;
    s.fault_code = v.mc_fault_code;
    s.pid_pos_now = v.pid_pos_now;
    s.controller_id = v.app_controller_id;
    s.ntc_temp_mos1 = v.temp_mos1;
    s.ntc_temp_mos2 = v.temp_mos2;
    s.ntc_temp_mos3 = v.temp_mos3;
    s.avg_vd = v.avg_vd;
    s.avg_vq = v.avg_vq;
    state_pub_->publish(msg);
  }

  rclcpp::Clock steady_;
  std::string port_;
  int64_t baud_;
  bool duty_mode_;
  double erpm_gain_, erpm_offset_, speed_min_, speed_max_;
  double max_speed_, duty_max_, duty_deadband_;
  bool invert_motor_;
  double servo_gain_, servo_offset_, servo_min_, servo_max_;
  double cmd_timeout_;
  int64_t read_timeout_ms_;

  int fd_ = -1;
  std::optional<AckermannDriveStamped> last_cmd_;
  std::optional<rclcpp::Time> last_cmd_time_;
  std::optional<double> last_servo_;

  rclcpp::Publisher<VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<RobocarVesc>();
    rclcpp::spin(node);
  }  // destructeur : duty 0, servo au neutre, port fermé
  rclcpp::shutdown();
  return 0;
}
