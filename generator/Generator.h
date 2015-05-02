#include <string>
#include <fstream>

class Generator{

public:
  void Parse(std::string path = "..\\monitor\\" ){
    std::ofstream myfile;
    myfile.open(path + "example.txt");
    myfile << "Writing this to a file.\n";
    myfile.close();
  }
  
  void Generate(std::string path = "..\\Generated\\"){
    std::ofstream myfile;
    myfile.open(path + "example.txt");
    myfile << "Writing this to a file.\n";
    myfile.close();
  }

};