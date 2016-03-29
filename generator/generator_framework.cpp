#include "generator_framework.h"

int monitor_generator::error_code = 0;
bool monitor_generator::wait_for_key_on_exit = false;

int monitor_generator::getErrorCode()
{
  return error_code;
}

void monitor_generator::setErrorCode(int error_code)
{
  error_code = error_code;
}

void monitor_generator::terminate()
{
  terminate(getErrorCode());
}

void monitor_generator::terminate(int error_code)
{
  BOOST_LOG_TRIVIAL(info) << std::string("Terminating. Error value: ") + std::to_string(error_code);

  if (wait_for_key_on_exit) {
    BOOST_LOG_TRIVIAL(info) << "Press any key to quit.";
    getchar();
  }
  std::exit(error_code);
}
