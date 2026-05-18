#include <iostream>

#include "ballistics.hpp"

using namespace std;

int main(int argc, char **argv) {
  if (argc != 2) {
    cerr << "Usage: ballistics_cli <input_file_path>" << endl;
    return 1;
  }

  const string INPUT_FILE = argv[1];

  BallisticsInput input = parseInputFile(INPUT_FILE);

  DropSolution dropSolution = computeDropSolution(input);

  if (!dropSolution.errorMessage.empty()) {
    cerr << "Error: " << dropSolution.errorMessage << endl;
    return 1;
  }

  writeOutputFile("output.txt", dropSolution);
}