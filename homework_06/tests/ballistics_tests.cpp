#include <gtest/gtest.h>

#include <cmath>

#include "ballistics.hpp"

const Ammo TEST_AMMO = {
    .name = "VOG-17",
    .mass = 0.35f,
    .drag = 0.07f,
    .lift = 0.0f};

TEST(BallisticsTests, TestComputesKnownDropPoint) {
  BallisticsInput input = parseInputFile(TEST_DATA_DIR "/sample_vog17.txt");

  const DropSolution solution = computeDropSolution(input);

  EXPECT_NEAR(solution.fireX, 173.759, 0.01);
  EXPECT_NEAR(solution.fireY, 173.759, 0.01);
}

TEST(BallisticsTests, TestComputesUnknownAmmo) {
  BallisticsInput input = parseInputFile(TEST_DATA_DIR "/unknown_ammo.txt");

  const DropSolution solution = computeDropSolution(input);

  EXPECT_EQ(solution.errorMessage, "Unknown ammo type: F1");
}

TEST(BallisticsTests, TestCalcFallTime) {
  float attackSpeed = 50.0f;
  float droneHeight = 100.0f;

  float fallTime = calcAmmoFallTime(TEST_AMMO, attackSpeed, droneHeight);

  EXPECT_NEAR(fallTime, 5.74f, 0.01f);
}

TEST(BallisticsTests, TestCalcHorizontalDistance) {
  DroneConfig drone = {
      .startPos = {10.0f, 10.0f, 100.0f},
      .ammo = TEST_AMMO,
      .attackSpeed = 50.0f};

  drone.attackSpeed = 50.0f;

  float fallTime = calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z);

  Coord targetPosition = {100.0f, 100.0f, 0.};

  float horizontalDistance = calcHorizontalDistance(fallTime, drone, targetPosition);

  EXPECT_NEAR(horizontalDistance, 185.5f, 0.06f);
}

TEST(BallisticsTests, TestCalcDistance) {
  Coord a = {0.0f, 0.0f};
  Coord b = {3.0f, 4.0f};

  float distance = calcDistance(a, b);

  EXPECT_NEAR(distance, 5.0f, 0.001f);
}

TEST(BallisticsTests, TestCalcFireCoordinates) {
  float horizontalDistance = 185.5f;
  float distanceToTarget = 190.0f;
  float xd = 10.0f;
  float yd = 10.0f;
  float targetX = 100.0f;
  float targetY = 0.0f;

  Coord fireCoords = calcFireCoordinates(horizontalDistance, distanceToTarget, xd, yd, targetX, targetY);

  EXPECT_NEAR(fireCoords.x, 12.1f, 0.1f);
  EXPECT_NEAR(fireCoords.y, 9.7f, 0.1f);
}