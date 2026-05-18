#define _USE_MATH_DEFINES
#include "ballistics.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;

const float GRAVITY_ACCEL = 9.81f;
const int AMMO_TYPES_COUNT = 5;

const Ammo ARSENAL[AMMO_TYPES_COUNT] = {
    {.name = "VOG-17", .mass = 0.35f, .drag = 0.07f, .lift = 0.0f},
    {.name = "M67", .mass = 0.6f, .drag = 0.1f, .lift = 0.0f},
    {.name = "RKG-3", .mass = 1.2f, .drag = 0.1f, .lift = 0.0f},
    {.name = "GLIDING-VOG", .mass = 0.45f, .drag = 0.1f, .lift = 1.0f},
    {.name = "GLIDING-RKG", .mass = 1.4f, .drag = 0.1f, .lift = 1.0f},
};

float calcAmmoFallTime(const Ammo &ammo, const float &attackSpeed,
                       const float &droneHeight) {
  /*
    V₀ — швидкість атаки дрона
    Z₀ — висота дрона (zd)
    g = 9.81 м/с²

    a = d·g·m − 2d²·l·V₀
    b = −3g·m² + 3d·l·m·V₀
    c = 6m²·Z₀
  */

  float drag2 = ammo.drag * ammo.drag;
  float mass2 = ammo.mass * ammo.mass;
  float a = ammo.drag * GRAVITY_ACCEL * ammo.mass -
            2.0f * drag2 * ammo.lift * attackSpeed;
  float b = -3.0f * GRAVITY_ACCEL * mass2 +
            3.0f * ammo.drag * ammo.lift * ammo.mass * attackSpeed;
  float c = 6.0f * mass2 * droneHeight;

  /*
    p = − b² / (3a²)
    q = 2b³ / (27a³) + c / a
    φ = arccos( 3q / (2p) · √(−3/p) )
    t = 2√(−p/3) · cos( (φ + 4π) / 3 ) − b / (3a)
  */

  float a2 = a * a;
  float a3 = a2 * a;
  float b2 = b * b;
  float b3 = b2 * b;

  float p = -b2 / static_cast<double>(3.0 * a2);
  float q =
      2.0 * b3 / static_cast<double>(27.0 * a3) + c / static_cast<double>(a);
  float phi = acos(3.0 * q / static_cast<double>(2.0 * p) *
                   sqrt(-3.0 / static_cast<double>(p)));
  float t = 2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) -
            b / static_cast<double>(3.0 * a);

  return t;
}

float calcHorizontalDistance(const float &fallTime, DroneConfig &drone,
                             const Coord &targetPosition) {
  /*
    h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
    + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀) / (36(1+l²)²m³)
    + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
  */
  float t = fallTime;
  float drag = drone.ammo.drag;
  float lift = drone.ammo.lift;
  float mass = drone.ammo.mass;
  float attackSpeed = drone.attackSpeed;

  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t3 * t;
  float t5 = t4 * t;
  float drag2 = drag * drag;
  float drag3 = drag2 * drag;
  float drag4 = drag3 * drag;
  float lift2 = lift * lift;
  float lift3 = lift2 * lift;
  float lift4 = lift3 * lift;
  float mass2 = mass * mass;
  float mass3 = mass2 * mass;
  float mass4 = mass3 * mass;

  float h =
      attackSpeed * t - t2 * drag * attackSpeed / (2.0 * mass) +
      t3 *
          (6.0 * drag * GRAVITY_ACCEL * lift * mass -
           6.0 * drag2 * (lift2 - 1.0) * attackSpeed) /
          (36.0 * mass2) +
      t4 *
          (-6.0 * drag2 * GRAVITY_ACCEL * lift * (1.0 + lift2 + lift4) * mass +
           3.0 * drag3 * lift2 * (1.0 + lift2) * attackSpeed +
           6.0 * drag3 * lift4 * (1.0 + lift2) * attackSpeed) /
          (36.0 * pow(1.0 + lift2, 2) * mass3) +
      t5 *
          (3.0 * drag3 * GRAVITY_ACCEL * lift3 * mass -
           3.0 * drag4 * lift2 * (1.0 + lift2) * attackSpeed) /
          (36.0 * (1.0 + lift2) * mass4);

  // CHECK IF DRONE IS PRECISELY OVER THE TARGET SO DISTANCE IS NOT ZERO ------
  if (targetPosition.x == drone.startPos.x &&
      targetPosition.y == drone.startPos.y) {
    drone.startPos.x = drone.startPos.x - 0.0001f;
    drone.startPos.y = drone.startPos.y - 0.0001f;
  }

  return h;
}

float calcDistance(const Coord &a, const Coord &b) {
  /*
    D = √( (targetX − xd)² + (targetY − yd)² )
  */

  float distance = sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));

  return distance;
}

Coord calcFireCoordinates(const float &horizontalDistance,
                          const float &distanceToTarget, const float &xd,
                          const float &yd, const float &targetX,
                          const float &targetY) {
  /*
    ratio = (D − h) / D
    fireX = xd + (targetX − xd) · ratio
    fireY = yd + (targetY − yd) · ratio
  */

  float ratio = (distanceToTarget - horizontalDistance) /
                static_cast<double>(distanceToTarget);
  float fireX = xd + (targetX - xd) * ratio;
  float fireY = yd + (targetY - yd) * ratio;

  return {fireX, fireY};
}

bool isManoeuvreNeeded(const float &horizontalDistance,
                       const float &accelerationPath,
                       const float &distanceToTarget) {
  /*
    Manoeuvre is needed if (accelerationPath + horizontalDistance) >
    distanceToTarget
  */

  return (accelerationPath + horizontalDistance) > distanceToTarget;
}

void processManouvre(DroneConfig &drone, const Coord &targetPosition) {
  /*
    xd' = targetX − (targetX − xd) · (h + accelerationPath) / D
    yd' = targetY − (targetY − yd) · (h + accelerationPath) / D
  */

  float h = calcHorizontalDistance(
      calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z), drone,
      targetPosition);
  float distanceToTarget = calcDistance(drone.startPos, targetPosition);

  if (isManoeuvreNeeded(h, drone.accelPath, distanceToTarget)) {
    drone.startPos.x =
        targetPosition.x - (targetPosition.x - drone.startPos.x) *
                               (h + drone.accelPath) / distanceToTarget;
    drone.startPos.y =
        targetPosition.y - (targetPosition.y - drone.startPos.y) *
                               (h + drone.accelPath) / distanceToTarget;
  }
}

BallisticsInput parseInputFile(const string &filePath) {
  ifstream inputFile(filePath);

  if (!inputFile.is_open()) {
    cerr << "Error: Could not open the file:" << filePath << endl;
    exit(1);
  }

  BallisticsInput input;

  inputFile >> input.droneX >> input.droneY >> input.droneZ >> input.targetX >>
      input.targetY >> input.attackSpeed >> input.accelerationPath >>
      input.ammoName;

  inputFile.close();

  return input;
}

DropSolution computeDropSolution(const BallisticsInput &input) {
  DroneConfig drone = {
      .startPos = {.x = input.droneX, .y = input.droneY, .z = input.droneZ},
      .attackSpeed = input.attackSpeed,
      .accelPath = input.accelerationPath};

  for (int i = 0; i < AMMO_TYPES_COUNT; i++) {
    if (input.ammoName == ARSENAL[i].name) {
      drone.ammo = ARSENAL[i];
      break;
    }
  }

  if (drone.ammo.name == "Unknown") {
    return {.errorMessage = "Unknown ammo type: " + input.ammoName};
  }

  cout << "AMMO TYPE: " << drone.ammo.name << endl;

  float fallTime =
      calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z);

  cout << "Fall time: " << fallTime << "s" << endl;

  Coord targetPosition = {input.targetX, input.targetY};

  float horizontalDistance =
      calcHorizontalDistance(fallTime, drone, targetPosition);

  cout << "Horizontal distance: " << horizontalDistance << "m" << endl;

  float distanceToTarget = calcDistance(drone.startPos, targetPosition);

  cout << "Distance to target: " << distanceToTarget << "m" << endl;

  DropSolution dropSolution{};

  if (isManoeuvreNeeded(horizontalDistance, drone.accelPath,
                        distanceToTarget)) {
    cout << endl << "Manoeuvre is needed!" << endl;

    dropSolution.isManoeuvrePerformed = true;
    dropSolution.manouvreX = drone.startPos.x;
    dropSolution.manouvreY = drone.startPos.y;

    processManouvre(drone, targetPosition);
    horizontalDistance = calcHorizontalDistance(
        calcAmmoFallTime(drone.ammo, drone.attackSpeed, drone.startPos.z),
        drone, targetPosition);
    distanceToTarget = calcDistance(drone.startPos, targetPosition);

    cout << "Drone moves to new x: " << drone.startPos.x
         << " y: " << drone.startPos.y << endl
         << endl;
  }

  Coord fireCoordinates = calcFireCoordinates(
      horizontalDistance, distanceToTarget, drone.startPos.x, drone.startPos.y,
      targetPosition.x, targetPosition.y);

  cout << endl
       << "Fire x: " << fireCoordinates.x << " y: " << fireCoordinates.y
       << endl;

  dropSolution.fireX = fireCoordinates.x;
  dropSolution.fireY = fireCoordinates.y;

  return dropSolution;
}

void writeOutputFile(const string &filePath, const DropSolution &dropSolution) {
  ofstream outputFile(filePath);

  if (!outputFile.is_open()) {
    cerr << "Error: Could not open the file for writing:" << filePath << endl;
    exit(1);
  }

  if (dropSolution.isManoeuvrePerformed) {
    outputFile << dropSolution.manouvreX << " " << dropSolution.manouvreY
               << " ";
  }

  outputFile << dropSolution.fireX << " " << dropSolution.fireY;

  outputFile.close();
}
