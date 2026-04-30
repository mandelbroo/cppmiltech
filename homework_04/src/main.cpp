#define _USE_MATH_DEFINES
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace std;

const float TICKS_PER_REVOLUTION = 1024.0;  // 1024 імпульсів на оберт колеса
const float WHEEL_RADIUS_M = 0.3;           // 30 см
const float WHEELBASE_M = 1.0;              // 1 м відстань між передньою та задньою віссю
const string OUTPUT_FILE_PATH = "output.txt";

int main(int argc, char** argv) {
  if (argc != 2) {
    cerr << "Usage: ugv_odometry <input_path>" << endl;
    return 1;
  }

  fstream inputFile(argv[1], ios::in);

  if (!inputFile.is_open()) {
    cerr << "Error: Could not open the file:" << argv[1] << endl;
    return 1;
  }

  fstream outputFile(OUTPUT_FILE_PATH, ios::out);

  if (!outputFile.is_open()) {
    cerr << "Error: Could not open the output file:" << OUTPUT_FILE_PATH << endl;
    return 1;
  }

  int timestamp_ms;
  bool isFirstStep = true;
  float x = 0, y = 0, theta = 0;
  float fl_ticks, fr_ticks, bl_ticks, br_ticks;
  float prev_fl_ticks, prev_fr_ticks, prev_bl_ticks, prev_br_ticks;
  string line;

  while (getline(inputFile, line)) {
    istringstream ss(line);
    ss >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks;

    if (isFirstStep) {
      isFirstStep = false;
      prev_fl_ticks = fl_ticks;
      prev_fr_ticks = fr_ticks;
      prev_bl_ticks = bl_ticks;
      prev_br_ticks = br_ticks;
      outputFile << timestamp_ms << " " << x << " " << y << " " << theta << endl;

      continue;  // Пропускаємо перший елемент оскільки нам потрібні два елементи для обчислення дельти
    }

    // Крок 1. Delta iмпульсiв по кожному колесу:
    float d_fl = fl_ticks - prev_fl_ticks;
    float d_fr = fr_ticks - prev_fr_ticks;
    float d_bl = bl_ticks - prev_bl_ticks;
    float d_br = br_ticks - prev_br_ticks;

    // Крок 2. Усереднити борти (передне i заднє колесо одного боку обертаються синхронно):
    float d_left = (d_fl + d_bl) / 2.0;
    float d_right = (d_fr + d_br) / 2.0;

    // Крок 3. Перевести iмпульси у метри:
    float distance_per_tick = 2 * M_PI * WHEEL_RADIUS_M / TICKS_PER_REVOLUTION;
    float dL = d_left * distance_per_tick;
    float dR = d_right * distance_per_tick;

    // Крок 4. Скiльки пройшов центр робота i на скiльки повернувся:
    float d = (dL + dR) / 2.0;               // пройдена вiдстань центру
    float dtheta = (dR - dL) / WHEELBASE_M;  // змiна орiєнтацiї

    // Крок 5. Оновити позицiю (midpoint integration - усереднений напрямок на кроцi):
    x += d * cos(theta + dtheta / 2.0);
    y += d * sin(theta + dtheta / 2.0);
    theta += dtheta;

    prev_fl_ticks = fl_ticks;
    prev_fr_ticks = fr_ticks;
    prev_bl_ticks = bl_ticks;
    prev_br_ticks = br_ticks;
    outputFile << timestamp_ms << " " << x << " " << y << " " << theta << endl;
  }

  inputFile.close();
  outputFile.close();

  cout << "Odometry calculation completed. Output written to: " << OUTPUT_FILE_PATH << endl;

  return 0;
}
