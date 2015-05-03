#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

using namespace std;
using namespace boost::filesystem;

class Generator{

public:
  void Parse(std::string path = "..\\monitor\\" ){

  }
  
  void Generate(std::string generatedPath_ = "..\\Generated\\", std::string sourcePath_ = "..\\monitor\\"){
    path sourcePath(sourcePath_);
    path generatedPath(generatedPath_);
    directory_iterator end_itr;

    string propertyFileString;
    
    //Copy the source files from the monitor directory to the Generated directory
    for (directory_iterator iterator(sourcePath_); iterator != end_itr; ++iterator){
        
        string fileName = iterator->path().filename().string();
        directory_entry entry(*iterator);
        
        path generatedPathTemp(generatedPath);
        generatedPathTemp += fileName;
        copy_file(entry.path(), generatedPathTemp, copy_option::overwrite_if_exists);
        if (fileName == "Property.h"){
          wstring temp = entry.path().native();
          ifstream tempFile(temp.c_str());

          std::string str((std::istreambuf_iterator<char>(tempFile)),
            std::istreambuf_iterator<char>());
          propertyFileString = move(str);
        }
    }
  }
  /*
  Eval function example (operator overloads also work!).
  Replace the //--EVALFUNCTIONS-- substring with the generated string!

  Add the function to the //--DECLARATIONS-- 
  trilean EVAL_s1a(Property* _prop)
  {
    return NAND_3( 
        NOT_3(_prop->isEventFired(EVENT_B)), NAND_3(_prop->isEventFired(EVENT_C), _prop->InputStates()[1])
        );
  }
  */

  /*
  Construct.
  Replace the //--CONSTRUCTFUNCTIONS-- substring with the generated string
  Add the function to the //--DECLARATIONS-- 

  Just set the input and output sizes, evalfunctions and construct children node function.
  Example:  
  Property* constructS0(Property* _rootNode)
  {
    _rootNode->evalFunctions.push_back(EVAL_s0);
    _rootNode->constructChildrenNodeFunc = constructS1;
    _rootNode->outputStates.resize(1);
    _rootNode->inputStates.resize(2);
    return _rootNode;
  }  
  */

};