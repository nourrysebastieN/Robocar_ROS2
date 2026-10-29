// Tests de la logique de conduite (mapping jeu de voiture), sans manette.
#include <gtest/gtest.h>

#include "robocar_teleop/drive_ramp.hpp"

using robocar_teleop::DriveRamp;
using robocar_teleop::should_publish_teleop;

namespace
{

// Appelle update() n fois avec les mêmes gâchettes
void run(DriveRamp & r, int n, double gas, double brake)
{
  for (int i = 0; i < n; i++) {
    r.update(gas, brake);
  }
}

TEST(DriveRamp, RtAcceleratesSlowlyWithExpo)
{
  DriveRamp r;
  r.update(1.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.029);  // accel_step
  EXPECT_DOUBLE_EQ(r.target(), 1.0);
  run(r, 100, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 1.0);
  run(r, 100, 0.5, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.25);   // 0,5² : expo 2
}

TEST(DriveRamp, ReleaseRtDeceleratesToZero)
{
  DriveRamp r;
  run(r, 100, 1.0, 0.0);
  r.update(0.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 1.0 - 0.071);  // decel_step
  run(r, 100, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  EXPECT_FALSE(r.reversing());
}

TEST(DriveRamp, LtBrakesHarderWhenPressedMore)
{
  DriveRamp light, hard;
  run(light, 100, 1.0, 0.0);
  run(hard, 100, 1.0, 0.0);
  light.update(0.0, 0.1);
  hard.update(0.0, 1.0);
  EXPECT_TRUE(hard.braking());
  EXPECT_LT(hard.speed(), light.speed());
  EXPECT_NEAR(hard.speed(), 1.0 - 0.286 * 3.0, 1e-9);
  EXPECT_GE(light.speed(), 0.0);
}

TEST(DriveRamp, LtHeldAfterStopReversesAfterPause)
{
  DriveRamp r;
  run(r, 100, 1.0, 0.0);
  run(r, 2, 0.0, 1.0);                  // 1 - 2 × 0,858 < 0 : arrêté au 2e pas
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  run(r, 4, 0.0, 1.0);                  // pause au point mort (cross_hold_ticks)
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  EXPECT_TRUE(r.reversing());           // la cible est déjà en arrière
  r.update(0.0, 1.0);
  EXPECT_DOUBLE_EQ(r.speed(), -0.029);  // recule, avec la rampe d'accélération
  run(r, 100, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(r.speed(), -1.0);
}

TEST(DriveRamp, LtFromStandstillReversesImmediately)
{
  DriveRamp r;
  r.update(0.0, 1.0);
  EXPECT_DOUBLE_EQ(r.speed(), -0.029);
}

TEST(DriveRamp, RtWhileReversingBrakesThenGoesForward)
{
  DriveRamp r;
  run(r, 100, 0.0, 1.0);
  ASSERT_DOUBLE_EQ(r.speed(), -1.0);
  r.update(1.0, 0.0);
  EXPECT_TRUE(r.braking());
  EXPECT_DOUBLE_EQ(r.speed(), -1.0 + 0.357);  // cross_step
  run(r, 2, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  run(r, 4, 1.0, 0.0);                        // pause
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  r.update(1.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.029);
}

TEST(DriveRamp, NeverCrossesZeroWithoutPause)
{
  DriveRamp r;
  double prev = 0.0;
  // Alterne RT et LT au hasard : jamais de passage direct de + à - ou de - à +
  for (int i = 0; i < 2000; i++) {
    const bool gas = (i / 37) % 2 == 0;
    r.update(gas ? 1.0 : 0.0, gas ? 0.0 : 0.7);
    EXPECT_FALSE((prev > 0.0 && r.speed() < 0.0) || (prev < 0.0 && r.speed() > 0.0)) << i;
    prev = r.speed();
  }
}

TEST(DriveRamp, ResetStopsAndClearsPause)
{
  DriveRamp r;
  run(r, 50, 1.0, 0.0);
  r.reset();
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  EXPECT_FALSE(r.moving());
  r.update(1.0, 0.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.029);
}

TEST(DriveRamp, MovingDuringPause)
{
  DriveRamp r;
  run(r, 100, 1.0, 0.0);
  run(r, 2, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(r.speed(), 0.0);
  EXPECT_TRUE(r.moving());  // pause en cours : la manette garde la main
}

TEST(Mux, TeleopSilentOnlyWhenDriveEnabledAndIdle)
{
  EXPECT_TRUE(should_publish_teleop(false, false));  // rien : /teleop à 0, /drive bloqué
  EXPECT_TRUE(should_publish_teleop(true, false));   // conduite manuelle
  EXPECT_FALSE(should_publish_teleop(false, true));  // RB seul : /drive passe
  EXPECT_TRUE(should_publish_teleop(true, true));    // RB + manette : la manette reprend
}

}  // namespace
