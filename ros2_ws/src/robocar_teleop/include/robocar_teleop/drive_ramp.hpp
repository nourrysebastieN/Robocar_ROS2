// Logique de conduite de robocar_teleop, sans ROS ni libevdev (testable seule).
//
// Mapping « jeu de voiture » : RT accélère, LT freine, et LT maintenu à l'arrêt recule.
// Rampe reprise de soukoukou (le_loto/RoboCar/src/control/speed.c) : lente en accélération,
// rapide en décélération, passage par zéro avec une pause au point mort.
// Unités : fraction signée de max_speed (+ = avant), un appel à update() par pas de rampe.
#ifndef ROBOCAR_TELEOP__DRIVE_RAMP_HPP_
#define ROBOCAR_TELEOP__DRIVE_RAMP_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robocar_teleop
{

struct DriveRampParams
{
  double throttle_curve = 2.0;  // expo des gâchettes : > 1 = plus fin à basse vitesse
  double accel_step = 0.029;    // par pas, en s'éloignant de 0
  double decel_step = 0.071;    // par pas, en revenant vers 0 (gâchette relâchée)
  double brake_step = 0.286;    // par pas, LT en marche avant, × (1 + 2 × LT)
  double cross_step = 0.357;    // par pas, RT en marche arrière (freinage avant de repartir)
  int64_t cross_hold_ticks = 4; // pause au point mort avant de changer de sens
};

class DriveRamp
{
public:
  using Params = DriveRampParams;

  DriveRamp() = default;
  explicit DriveRamp(const Params & p) : p_(p) {}

  // gas = RT, brake = LT, de 0 à 1
  void update(double gas, double brake)
  {
    braking_ = false;
    if (brake > 0.0 && speed_ > 0.0) {
      // En avant : LT freine, d'autant plus fort qu'il est enfoncé
      braking_ = true;
      target_ = 0.0;
      speed_ = std::max(0.0, speed_ - p_.brake_step * (1.0 + 2.0 * brake));
      if (speed_ == 0.0) {
        cross_hold_ = p_.cross_hold_ticks;  // arrêté : pause avant de reculer
      }
      return;
    }
    if (brake > 0.0) {
      target_ = -curve(brake);  // à l'arrêt ou en arrière : LT recule
    } else if (gas > 0.0) {
      target_ = curve(gas);
    } else {
      target_ = 0.0;
    }

    if (cross_hold_ > 0) {
      cross_hold_--;
      speed_ = 0.0;
      return;
    }
    if (speed_ < 0.0 && target_ > 0.0) {
      // En arrière : RT freine vite jusqu'à 0, pause, puis repart en avant
      braking_ = true;
      if (-speed_ <= p_.cross_step) {
        speed_ = 0.0;
        cross_hold_ = p_.cross_hold_ticks;
      } else {
        speed_ += p_.cross_step;
      }
      return;
    }
    const double step = std::abs(target_) > std::abs(speed_) ? p_.accel_step : p_.decel_step;
    speed_ = step_toward(speed_, target_, step);
  }

  void reset()
  {
    speed_ = 0.0;
    target_ = 0.0;
    braking_ = false;
    cross_hold_ = 0;
  }

  double speed() const {return speed_;}
  double target() const {return target_;}
  bool braking() const {return braking_;}
  bool moving() const {return speed_ != 0.0 || cross_hold_ > 0;}
  bool reversing() const {return speed_ < 0.0 || (speed_ == 0.0 && target_ < 0.0);}

private:
  double curve(double v) const {return std::pow(std::clamp(v, 0.0, 1.0), p_.throttle_curve);}

  static double step_toward(double value, double target, double step)
  {
    if (std::abs(target - value) <= step) {
      return target;
    }
    return value + (target > value ? step : -step);
  }

  Params p_;
  double speed_ = 0.0;
  double target_ = 0.0;
  bool braking_ = false;
  int64_t cross_hold_ = 0;
};

// Faut-il publier /teleop ? Non quand RB est maintenu et que la manette ne conduit pas :
// sinon /teleop (priorité 100), même à 0, masquerait /drive (10) dans le mux.
inline bool should_publish_teleop(bool manual_active, bool drive_enabled)
{
  return manual_active || !drive_enabled;
}

}  // namespace robocar_teleop

#endif  // ROBOCAR_TELEOP__DRIVE_RAMP_HPP_
