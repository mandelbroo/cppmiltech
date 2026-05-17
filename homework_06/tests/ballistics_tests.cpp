#include <gtest/gtest.h>
#include <cmath>

#include "ballistics.hpp"

TEST(BallisticsTests, TestCalcFallTime) {
  DroneConfig drone;

  drone.ammo.mass = 1.0f;
  drone.ammo.drag = 0.5f;
  drone.startPos = {0.0f, 0.0f, 100.0f};
  
  float fallTime = calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z);

  EXPECT_NEAR(std::round(fallTime * 1000.0f) / 1000.0f, 4.515f, 0.001f);
}

TEST(BallisticsTests, TestCalcHorizontalDistance) {
  DroneConfig drone;

  drone.ammo.mass = 1.0f;
  drone.ammo.drag = 0.5f;
  drone.startPos = {0.0f, 0.0f, 100.0f};
  drone.attackSpeed = 50.0f;

  float fallTime = calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z);
  Coord targetPosition = {100.0f, 0.0f};
  float horizontalDistance = calcHorizontalDistance(fallTime, drone, targetPosition);
  EXPECT_NEAR(horizontalDistance, 224.5f, 0.1f);
}

TEST(BallisticsTests, TestCalcDistance) {
  Coord a = {0.0f, 0.0f};
  Coord b = {3.0f, 4.0f};

  float distance = calcDistance(a, b);
  EXPECT_NEAR(distance, 5.0f, 0.001f);
}
