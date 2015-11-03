#include <iostream>
#include <boost/log/trivial.hpp>

#include "Generator.h"

int main(int argc, char* argv[]) {
  BOOST_LOG_TRIVIAL(info) << "ROS runtime monitor tool.";
  BOOST_LOG_TRIVIAL(info) << "Made by Bence Gazder.";

  //Example: std::string input = "G (((8 | 9) ^ 4) U (1 & 2))\n";
  std::string input = "G(1 => (2 U 3))";

  std::string monitor_source_path = "D:\\Projects\\R5-COP-Verification\\monitor";
  std::string monitor_destination_path = "D:\\Projects\\R5-COP-Verification\\generated";

  //cout << "Please enter the expression!" << endl;
  //getline(std::cin, input);
  Generator generator;
  generator.setMonitorDestinationPath(monitor_destination_path);
  generator.setMonitorSourcePath(monitor_source_path);
  generator.generateMonitor(input);

  BOOST_LOG_TRIVIAL(info) << "Press enter to quit.";
  getchar();

  return 0;
}