#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char** argv) {
  if (argc != 2) {
    cerr << "usage: ugv_odometry <input_path>" << endl;
    return 1;
  }

  ifstream file(argv[1], ios::in);

  if (!file.is_open()) {
    cerr << "Error: Could not open the file:" << argv[1] << endl;
    return 1;
  }

  float x = 0, y = 0, theta = 0;
  float timestamp_ms;
  float fl_ticks;
  float fr_ticks;
  float bl_ticks;
  float br_ticks;

  string line;

  while (getline(file, line)) {
    istringstream ss(line);
    ss >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks;

    cout << timestamp_ms << " " << fl_ticks << " " << fr_ticks << " " << bl_ticks
         << " " << br_ticks << endl;
  }
  file.close();

  //   int ticks_per_revolution = 1024;  // 1024 ticks per wheel revolution
  //   float wheel_radius_m = 0.3;       // 30 cm
  //   float wheelbase_m = 1.0;          // 1 m distance between front and rear axles
  //   float x = 0, y = 0, theta = 0;

  // TODO: implement wheel odometry for a 4-wheel differential-drive UGV.
  //
  // Parameters:
  //   ticks_per_revolution = 1024
  //   wheel_radius_m       = 0.3
  //   wheelbase_m          = 1.0
  //
  // Input:  text file with 5 whitespace-separated numbers per line:
  //         timestamp_ms fl_ticks fr_ticks bl_ticks br_ticks
  // Output: same tabular format on stdout, starting from the second sample:
  //         timestamp_ms x y theta

  return 0;
}
