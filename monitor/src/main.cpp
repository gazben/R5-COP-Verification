#include "monitor/monitor.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "--monitor_name--");
  Monitor::getInstance()->run();
  return 0;
}