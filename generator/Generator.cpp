#include "Generator.h"

void Generator::Generate(std::string sourcePath, std::string generatedPath)
{
  if (copyDir(boost::filesystem::path(sourcePath), boost::filesystem::path(generatedPath)))
    BOOST_LOG_TRIVIAL(info) << "Source code directory copied";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Error during the source code directory copying";

  if (propertyCppFilePath.empty())
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp is NOT found!";

  if (PropertyHeaderFilePath.empty())
    BOOST_LOG_TRIVIAL(fatal) << "Property.h is NOT found!";

  ifstream propertyCppFile_in(propertyCppFilePath);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.cpp is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp opening failed";
  propertyCppFileString = std::string((std::istreambuf_iterator<char>(propertyCppFile_in)), std::istreambuf_iterator<char>());

  ifstream peopertyHeaderFile(PropertyHeaderFilePath);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.h is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.h opening failed";
  propertyHeaderFileString = std::string((std::istreambuf_iterator<char>(peopertyHeaderFile)), std::istreambuf_iterator<char>());

  if (str_replace(propertyHeaderFileString, "//--DECLARATIONS--", blockGenerator->getFunctionDeclarations()))
    BOOST_LOG_TRIVIAL(info) << "Function declarations written to the Property.h file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Function declaration writing to the Property.h file failed!";

  if (str_replace(propertyCppFileString, "//--CONSTRUCTFUNCTIONS--", blockGenerator->getConstructFunctions()))
    BOOST_LOG_TRIVIAL(info) << "Construct functions written to the Property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Construct functions writing to the Property.cpp file failed!";

  if (str_replace(propertyCppFileString, "//--EVALFUNCTIONS--", blockGenerator->getFunctions()))
    BOOST_LOG_TRIVIAL(info) << "EvalFunctions written to the Property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "EvalFunction writing to the Property.cpp failed";

  ofstream propertyCppFile_out(propertyCppFilePath);
  if (propertyCppFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.cpp file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp fille opening for writing failed";
  propertyCppFile_out.write(propertyCppFileString.c_str(), propertyCppFileString.size());
  propertyCppFile_out.close();

  ofstream propertyHeaderFile_out(PropertyHeaderFilePath);
  if (propertyHeaderFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.h file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.h file opening for writing failed";
  propertyHeaderFile_out.write(propertyHeaderFileString.c_str(), propertyHeaderFileString.size());
  propertyHeaderFile_out.close();
}

Generator::Generator(BlockGenerator* _generator) :blockGenerator(_generator)
{
}

bool Generator::str_replace(std::string& str, const std::string& from, const std::string& to)
{
  unsigned int start_pos = str.find(from);
  if (start_pos == std::string::npos) {
    BOOST_LOG_TRIVIAL(error) << "String replace failed, \"" + from + "\" is not found in the: \" " + str.substr(10) + "...\" string";
    return false;
  }
  str.replace(start_pos, from.length(), to);
  return true;
}

bool Generator::copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination)
{
  namespace fs = boost::filesystem;
  try{
    // Check and create the directory
    if (fs::exists(destination)){
      BOOST_LOG_TRIVIAL(info) << "Destination directory " 
        << destination.string() << " already exists. Overriding...";
    }
    else {
      if (!fs::create_directory(destination)){
        BOOST_LOG_TRIVIAL(error) << "Unable to create destination directory "
          << destination.string();
        return false;
      }
    }
  }
  catch (fs::filesystem_error const & e){
    BOOST_LOG_TRIVIAL(error) << e.what();
    return false;
  }

  // Check and copy the files
  for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file){
    try{
      fs::path current(file->path());
      if (fs::is_directory(current)){
        if (!copyDir(current, destination / current.filename())){
          return false;
        }
      }
      else{
        if (current.filename() == "Property.h") {
          PropertyHeaderFilePath = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "Property.h found! Path: " << propertyCppFilePath;
        }

        if (current.filename() == "Property.cpp") {
          propertyCppFilePath = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "Property.cpp found! Path: " << propertyCppFilePath;
        }
        fs::copy_file(current, destination / current.filename(), copy_option::overwrite_if_exists);
      }
    }
    catch (fs::filesystem_error const & e){
      BOOST_LOG_TRIVIAL(error) << e.what();
    }
  }
  return true;
}