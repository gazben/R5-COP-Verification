#ifndef Generator_h__
#define Generator_h__

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Construct.h"
#include "Eval.h"


using namespace boost::filesystem;
using namespace std;

class Generator {
public:
  bool Parse(std::string input) {
    if (input.empty()) {
      //cout << "Empty input!" << endl;
      return false;
    }

    std::string::iterator iter = input.begin();
    std::string::iterator end_iter = input.end();

    return true;
  }

  void Generate(std::string generatedPath_ = "..\\..\\Generated\\", std::string sourcePath_ = "..\\..\\monitor\\src\\monitor\\") {
    path sourcePath(sourcePath_);
    path generatedPath(generatedPath_);
    directory_iterator end_itr;

    std::string propertyFileString;

    //Copy the source files from the monitor directory to the Generated directory
    for (directory_iterator iterator(sourcePath_); iterator != end_itr; ++iterator) {
      std::string fileName = iterator->path().filename().string();
      directory_entry entry(*iterator);

      path generatedPathTemp(generatedPath);
      generatedPathTemp += fileName;
      copy_file(entry.path(), generatedPathTemp, copy_option::overwrite_if_exists);
      if (fileName == "Property.h") {
        //wstring propertyFilePath = entry.path().native().c_str();
        ifstream tempFile( std::string( entry.path().native().c_str() ) );

        std::string str((std::istreambuf_iterator<char>(tempFile)),
          std::istreambuf_iterator<char>());
        propertyFileString = move(str);
      }
    }

    constructFunctions.push_back(ConstructFunction(vector<std::string>({ "func1", "func2" }), "ctorfunc", 2, 1));

    std::string functionDeclarations;
    std::string constructFunctionsString;
    std::string evalFunctionsString;

    for (unsigned int i = 0; i < constructFunctions.size(); i++) {
      functionDeclarations += constructFunctions[i].getDeclarationString();
      functionDeclarations += "\n";

      constructFunctionsString += constructFunctions[i].getFunctionString();
      constructFunctionsString += "\n";
    }

    for (unsigned int i = 0; i < evalFunctionsString.size(); i++) {
      functionDeclarations += evalFunctions[i].getDeclarationString();
      functionDeclarations += "\n";

      evalFunctionsString += evalFunctions[i].getFunctionString();
      evalFunctionsString += "\n";
    }

    str_replace(propertyFileString, "//--DECLARATIONS--", functionDeclarations);
    str_replace(propertyFileString, "//--CONSTRUCTFUNCTIONS--", constructFunctionsString);
    str_replace(propertyFileString, "//--EVALFUNCTIONS--", evalFunctionsString);

    ofstream propertyFile(generatedPath_ + "Property.h");
    propertyFile.write(propertyFileString.c_str(), propertyFileString.size());
    propertyFile.close();
  }

private:
  vector<ConstructFunction> constructFunctions;
  vector<EvalFunction> evalFunctions;

  bool str_replace(std::string& str, const std::string& from, const std::string& to) {
    unsigned int start_pos = str.find(from);
    if (start_pos == std::string::npos)
      return false;
    str.replace(start_pos, from.length(), to);
    return true;
  }
};
#endif
