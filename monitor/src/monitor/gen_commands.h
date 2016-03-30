#ifndef gen_commands_h__
#define gen_commands_h__

#include <string>

//True command
#ifndef DEBUG_NO_ROS
std::string true_command = "--true_command--";
std::string false_command = "--false_command--";
#else
std::string true_command = "echo result TRUE";
std::string false_command = "echo result FALSE";
#endif
//False command


#endif // gen_commands_h__
