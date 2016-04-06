#ifndef gen_commands_h__
#define gen_commands_h__

#include <string>

//True command
#ifndef DEBUG_NO_ROS
const std::string true_command = "--true_command--";
const std::string false_command = "--false_command--";
#else
const std::string true_command = "echo result TRUE";
const std::string false_command = "echo result FALSE";
#endif
//False command


#endif // gen_commands_h__
