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
        
        cout << generatedPathTemp << endl;
        copy_file(entry.path(), generatedPathTemp, copy_option::overwrite_if_exists);

        if (fileName == "Property.cpp"){
          cout << "Path: " + entry.path().string() << endl;

          wstring temp = entry.path().native();
          ifstream tempFile(temp.c_str());

          std::string str((std::istreambuf_iterator<char>(tempFile)),
            std::istreambuf_iterator<char>());
          propertyFileString = move(str);
        }
    }
    cout << propertyFileString << endl;
  }

};