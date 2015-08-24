#ifndef Generator_h__
#define Generator_h__

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>

#include "BlockGenerator.h"

using namespace boost::filesystem;
using namespace std;

class Generator {
public:

  std::string PropertyFilePath;
  std::string propertyFileString;

  std::string propertyHeaderFileString;
  std::string PropertyHeaderFilePath;

  void Generate(std::string sourcePath_ = "/home/ros/R5-COP-Verification/monitor", std::string generatedPath_ = "/home/ros/R5-COP-Verification/Generated");

  Generator(BlockGenerator* _generator);

private:
  BlockGenerator* blockGenerator;

  bool str_replace(std::string& str, const std::string& from, const std::string& to);

  bool copyDir(
    boost::filesystem::path const & source,
    boost::filesystem::path const & destination
    );
};
#endif
