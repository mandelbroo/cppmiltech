#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "ballistics.hpp"

using namespace std;

const int AMMO_TYPES_COUNT = 5;

const Ammo ARSENAL[AMMO_TYPES_COUNT] = {
    {.name = "VOG-17", .mass = 0.35f, .drag = 0.07f, .lift = 0.0f},
    {.name = "M67", .mass = 0.6f, .drag = 0.1f, .lift = 0.0f},
    {.name = "RKG-3", .mass = 1.2f, .drag = 0.1f, .lift = 0.0f},
    {.name = "GLIDING-VOG", .mass = 0.45f, .drag = 0.1f, .lift = 1.0f},
    {.name = "GLIDING-RKG", .mass = 1.4f, .drag = 0.1f, .lift = 1.0f},
};

int main(int argc, char** argv) {
  if (argc != 2) {
    cerr << "Usage: ballistics_cli <input_file_path>" << endl;
    return 1;
  }

  const string INPUT_FILE = argv[1];
  const float GRAVITY_ACCEL = 9.81f;

  // READ FILE AND INIT -------------------------------------------------------
  std::ifstream inputFile(INPUT_FILE);

  if (!inputFile.is_open()) {
    std::cerr << "Error: Could not open the file:" << INPUT_FILE << std::endl;
    return 1;
  }

  float xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath;
  std::string ammoName;

  inputFile >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> ammoName;

  float drag = 0.0f;
  float mass = 0.0f;
  float lift = 0.0f;

  DroneConfig drone = {{xd, yd, zd}, {}, attackSpeed, accelerationPath};

  for (int i = 0; i < AMMO_TYPES_COUNT; i++) {
    if (ammoName == ARSENAL[i].name) {
      drone.ammo = ARSENAL[i];
      break;
    }
  }

  cout << "AMMO TYPE: " << ammoName << endl;

  float t = calcAmmoFallTime(drone.ammo, attackSpeed, zd);

  cout << "Fall time: " << t << "s" << endl;

  float horizontalDistance = calcHorizontalDistance(t, drone, {targetX, targetY});

  float distance = calcDistance({xd, yd}, {targetX, targetY});

  cout << "Distance to target: " << distance << "m" << endl;

  // IS MANOEUVRE NEEDED? -----------------------------------------------------

  cout << "Horizontal distance: " << horizontalDistance << "m" << endl;

  bool isManoeuvrePerformed = false;
  float oldXd = xd;
  float oldYd = yd;

  if (isManoeuvreNeeded(horizontalDistance, accelerationPath, distance)) {
    cout << endl
         << "Manoeuvre is needed!" << endl;

    processManouvre(drone, {targetX, targetY});

    isManoeuvrePerformed = true;

    cout << "Drone moves to new x: " << drone.startPos.x << " y: " << drone.startPos.y << endl
         << endl;
  }

  Coord fireCoordinates = calcFireCoordinates(horizontalDistance, distance, oldXd, oldYd, targetX, targetY);

  cout << endl
       << "Fire x: " << fireCoordinates.x << " y: " << fireCoordinates.y << endl;

  // WRITE RESULTS INTO FILE --------------------------------------------------

  std::ofstream outputFile("output.txt");

  if (!outputFile.is_open()) {
    cout << "Error. Cannot open output file.";
    return 1;
  }

  if (isManoeuvrePerformed) {
    outputFile << drone.startPos.x << " " << drone.startPos.y << " ";
  }

  outputFile << fireCoordinates.x << " " << fireCoordinates.y;

  cout << endl;
}