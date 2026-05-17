#include <string>

struct Coord {
  double x;
  double y;
  double z = 0.0f;
};

struct Ammo {
  std::string name = "Unknown";
  float mass;  // маса (кг)
  float drag;  // коефіцієнт опору
  float lift;  // коефіцієнт підйому
};

struct DroneConfig {
  Coord startPos;  // початкова позиція (x, y)
  Ammo ammo;
  float altitude;     // висота
  float initialDir;   // початковий напрямок (рад)
  float attackSpeed;  // швидкість атаки (м/с)
  float accelPath;    // шлях розгону (м)
  // char ammoName[32];    // обрані боєприпаси
  float arrayTimeStep;  // крок часу масиву цілей
  float simTimeStep;    // крок симуляції
  float hitRadius;      // радіус влучення
  float angularSpeed;   // кутова швидкість (рад/с)
  float turnThreshold;  // поріг повороту (рад)
};

float calcAmmoFallTime(const Ammo& ammo, const float& attackSpeed, const float& droneHeight);
float calcDistance(const Coord& a, const Coord& b);
Coord calcFireCoordinates(const float& horizontalDistance, const float& distanceToTarget, const float& xd, const float& yd, const float& targetX, const float& targetY);
float calcHorizontalDistance(const float& fallTime, DroneConfig& drone, const Coord& targetPosition);
bool isManoeuvreNeeded(const float& horizontalDistance, const float& accelerationPath, const float& distanceToTarget);
void processManouvre(DroneConfig& drone, const Coord& targetPosition);