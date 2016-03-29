#ifndef generator_framework_h__
#define generator_framework_h__

/* GLOBAL INCLUDES */
#include <boost/log/trivial.hpp>
#include <string>

/* LOCAL INCLUDES */

/* INCLUDES END */

namespace monitor_generator {
  extern int error_code;
  extern bool wait_for_key_on_exit;

  int getErrorCode();
  void setErrorCode(int error_code);
  void terminate();
  void terminate(int error_code);
}


#endif // generator_framework_h__
