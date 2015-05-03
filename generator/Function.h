#ifndef Function_h__
#define Function_h__

#include <string>
#include <vector>

class Function{
public:

  Function(){
    ID = ID_counter++;
  }

  virtual string getFunctionString() = 0;
  virtual string getDeclarationString() = 0;

protected:
  virtual string getSignature() = 0;

  static unsigned int ID_counter;
  unsigned int ID;
};

#endif // Function_h__
