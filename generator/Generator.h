#ifndef Generator_h__
#define Generator_h__

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "Construct.h"

using namespace std;
using namespace boost::filesystem;

class Generator{
public:
  void Parse(std::string path = "..\\monitor\\"){
    //here comes the magic... eventually
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
        wstring propertyFilePath = entry.path().native();
        ifstream tempFile(propertyFilePath.c_str());

        std::string str((std::istreambuf_iterator<char>(tempFile)),
          std::istreambuf_iterator<char>());
        propertyFileString = move(str);
      }
    }

    constructFunctions.push_back(ConstructFunction(vector<string>({ "func1", "func2" }), "ctorfunc", 2, 1));

    string functionDeclarations;
    string constructFunctionsString;


    for (unsigned int i = 0; i < constructFunctions.size(); i++){
      functionDeclarations += constructFunctions[i].getDeclarationString();
      functionDeclarations += "\n";

      constructFunctionsString += constructFunctions[i].getFunctionString();
      constructFunctionsString += "\n";
    }
    str_replace(propertyFileString, "//--DECLARATIONS--", functionDeclarations);
    str_replace(propertyFileString, "//--CONSTRUCTFUNCTIONS--", constructFunctionsString);

    //Replace the //--EVALFUNCTIONS-- substring with the generated string!
    ofstream propertyFile(generatedPath_ + "Property.h");
    propertyFile.write(propertyFileString.c_str(), propertyFileString.size());
  }

private:
  vector<ConstructFunction> constructFunctions;

  bool str_replace(std::string& str, const std::string& from, const std::string& to) {
    unsigned int start_pos = str.find(from);
    if (start_pos == std::string::npos)
      return false;
    str.replace(start_pos, from.length(), to);
    return true;
  }
};
#endif // Generator_h__