#ifndef Function_h__
#define Function_h__

#include <string>
#include <vector>

class Function{
public:

  Function();

  virtual std::string getFunctionString() = 0;
  virtual std::string getDeclarationString() = 0;

protected:
  virtual std::string getSignature() = 0;

  static unsigned int ID_counter;
  unsigned int ID;
};

#endif // Function_h__
