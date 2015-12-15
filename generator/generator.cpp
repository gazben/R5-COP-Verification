#include "generator.h"

//STATIC init
boost::program_options::options_description Generator::arguments = boost::program_options::options_description();
boost::program_options::variables_map Generator::argument_variables = boost::program_options::variables_map();


Generator::Generator()
  :error_code(0)
{
  BOOST_LOG_TRIVIAL(info) << "ROS runtime monitor generator. Made by Bence Gazder";
  arguments.add_options()
    ("help", "See available options.")
    ("source-path", boost::program_options::value<std::string>()->required(), "Root directory of the monitor frame source.")
    ("output-path", boost::program_options::value<std::string>()->required(), "Root directory, of the generated monitor.")
    ("debug-output", boost::program_options::value<std::string>(), "Directory of the log files, and generation information.")
    ("input-expression", boost::program_options::value<std::string>()->required(), "Expression that defines, the monitor behavior.")
    ("monitor-name", boost::program_options::value<std::string>()->required(), "Name of the generated monitor.")
    ("true-command", boost::program_options::value<std::string>()->required(), "System call if the result is TRUE")
    ("false-command", boost::program_options::value<std::string>()->required(), "System call for the result is FALSE")
    ;
}

Generator::Generator(int argc, char* argv[]) :Generator()
{
  parseProgramArguments(argc, argv);
}

Generator::~Generator()
{
}

void Generator::run()
{
    try
    {
      /*
       * Ex.:
       *  - Linux: /home/user/projects/monitor_generator/monitor
       *  - Windows: D:\Projects\monitor_generator\monitor
       * */
      std::string monitor_source_path = argument_variables["source-path"].as<std::string>();
      /*
       * Ex.:
       *  - Linux:
       *  - Windows:
       * */
      std::string monitor_destination_path = argument_variables["output-path"].as<std::string>();

      /*
       * Ex.: std::string input = "G (((8 | 9) ^ 4) U (1 & 2))";
       */
      std::string input = argument_variables["input-expression"].as<std::string>();

      setMonitorDestinationPath(monitor_destination_path);
      setMonitorSourcePath(monitor_source_path);
      setExpressionInput(input);
      setRoot(parseInput(getExpressionInput()));

      auto tree_root = getRoot();
      AstOptimizer::optimizeAst(tree_root);   //remove the unnecessary parts of the AST
      block_generator.setAstRootNode(converter.convertToConnectionNormalForm(root));
      block_generator.createBlocks();
      generateMonitor();
      BOOST_LOG_TRIVIAL(info) << "Generation completed!";
      BOOST_LOG_TRIVIAL(info) << "Press enter to quit.";
    }
    catch (std::exception& e) {
      BOOST_LOG_TRIVIAL(fatal) << e.what();
    }
    catch (...) {
      //Unknown exception happened. Use some hack, to see what is it.
      std::exception_ptr eptr = std::current_exception();
      [&]() -> auto
      {
        try
        {
          if (eptr)
          {
            std::rethrow_exception(eptr);
          }
        }
        catch (const std::exception& e) {
          std::cout << e.what();
        }
      }();
    }
  terminate();
}

void Generator::setExpressionInput(std::string expression_input)
{
  BOOST_LOG_TRIVIAL(info) << "Expression set to: " << expression_input;
  this->expression_input = expression_input;
}

std::string Generator::getExpressionInput()
{
  return expression_input;
}

void Generator::setMonitorSourcePath(std::string monitor_source_path)
{
  BOOST_LOG_TRIVIAL(info) << "Monitor source path set to: " << monitor_source_path;
  this->monitor_source_path = monitor_source_path;
}

void Generator::setMonitorDestinationPath(std::string monitor_destination_path)
{
  BOOST_LOG_TRIVIAL(info) << "Generated monitor destination folder set to: " << monitor_destination_path;
  this->monitor_destination_path = monitor_destination_path;
}

void Generator::generateMonitor()
{
  if (copyDir(boost::filesystem::path(monitor_source_path), boost::filesystem::path(monitor_destination_path)))
    BOOST_LOG_TRIVIAL(info) << "Source code directory copied";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Error during the source code directory copying";

  if (property_cpp_file_path.empty())
    BOOST_LOG_TRIVIAL(fatal) << "property.cpp is NOT found!";

  if (property_header_file_path.empty())
    BOOST_LOG_TRIVIAL(fatal) << "property.h is NOT found!";

  std::ifstream propertyCppFile_in(property_cpp_file_path);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "property.cpp is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp opening failed";
  property_cpp_file_content = std::string((std::istreambuf_iterator<char>(propertyCppFile_in)), std::istreambuf_iterator<char>());

  std::ifstream propertyHeaderFile(property_header_file_path);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "property.h is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "property.h opening failed";
  property_header_file_content = std::string((std::istreambuf_iterator<char>(propertyHeaderFile)), std::istreambuf_iterator<char>());

  std::ifstream cmakeListsTxtFile(cmake_txt_file_path);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "CMakeLists.txt is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "CMakeLists.txt opening failed";
  cmake_txt_file_content = std::string((std::istreambuf_iterator<char>(cmakeListsTxtFile)), std::istreambuf_iterator<char>());

  while (str_replace(cmake_txt_file_content, "--monitor_name--", argument_variables["monitor-name"].as<std::string>()));
  std::ofstream cmakeListsTxtFile_out(cmake_txt_file_path);
  if (cmakeListsTxtFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "CMakeLists.txt file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "CMakeLists.txt fille opening for writing failed";
  cmakeListsTxtFile_out.write(cmake_txt_file_content.c_str(), cmake_txt_file_content.size());
  cmakeListsTxtFile_out.close();

  std::ifstream packageXmlFile(package_xml_file_path);
  if (packageXmlFile.is_open())
    BOOST_LOG_TRIVIAL(info) << "package.xml is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "package.xml opening failed";
  package_xml_file_content = std::string((std::istreambuf_iterator<char>(packageXmlFile)), std::istreambuf_iterator<char>());

  while (str_replace(package_xml_file_content, "--monitor_name--", argument_variables["monitor-name"].as<std::string>()));
  std::ofstream packageXml_out(package_xml_file_path);
  if (packageXml_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "package.xml file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "CMakeLists.txt fille opening for writing failed";
  packageXml_out.write(package_xml_file_content.c_str(), package_xml_file_content.size());
  packageXml_out.close();

  if (str_replace(property_header_file_content, "//--DECLARATIONS--", block_generator.getFunctionDeclarations()))
    BOOST_LOG_TRIVIAL(info) << "Function declarations written to the property.h file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Function declaration writing to the property.h file failed!";

  if (str_replace(property_cpp_file_content, "//--CONSTRUCTFUNCTIONS--", block_generator.getConstructFunctions()))
    BOOST_LOG_TRIVIAL(info) << "Construct functions written to the property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Construct functions writing to the property.cpp file failed!";

  if (str_replace(property_cpp_file_content, "//--EVALFUNCTIONS--", block_generator.getFunctions()))
    BOOST_LOG_TRIVIAL(info) << "EvalFunctions written to the property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "EvalFunction writing to the property.cpp failed";

  str_replace(property_cpp_file_content, "--true_command--", argument_variables["true-command"].as<std::string>());
  str_replace(property_cpp_file_content, "--false_command--", argument_variables["false-command"].as<std::string>());

  std::ofstream propertyCppFile_out(property_cpp_file_path);
  if (propertyCppFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "property.cpp file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "property.cpp fille opening for writing failed";
  propertyCppFile_out.write(property_cpp_file_content.c_str(), property_cpp_file_content.size());
  propertyCppFile_out.close();

  std::ofstream propertyHeaderFile_out(property_header_file_path);
  if (propertyHeaderFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "property.h file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "property.h file opening for writing failed";
  propertyHeaderFile_out.write(property_header_file_content.c_str(), property_header_file_content.size());
  propertyHeaderFile_out.close();
}

std::shared_ptr<base_rule::node> Generator::parseInput(std::string expression_input)
{
  std::shared_ptr<base_rule::node> result_root;
  base_rule::set_build_ast(true);
  base_rule::match_range context(expression_input.cbegin(), expression_input.cend());
  base_rule::match_range result_range;

  BOOST_LOG_TRIVIAL(info) << "Parsing the given expression...";
  if (ltl().match(context, result_range, result_root)) {
    BOOST_LOG_TRIVIAL(info) << "Expression parsed!";
  }
  else {
    BOOST_LOG_TRIVIAL(fatal) << "Expression parsing failed. Given expression is not valid!";
  }
  return result_root;
}

int Generator::getErrorCode()
{
  return error_code;
}

void Generator::setErrorCode(int error_code)
{
  this->error_code = error_code;
}

std::shared_ptr<base_rule::node> Generator::getRoot()
{
  return root;
}

void Generator::setRoot(std::shared_ptr<base_rule::node> root)
{
  this->root = root;
}

void Generator::parseProgramArguments(int argc, char* argv[])
{
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, arguments), argument_variables);
  
  if (!argument_variables["help"].empty() || argument_variables.empty())
  {
    BOOST_LOG_TRIVIAL(info) << "Displaying help options.";
    std::cout << arguments;
    terminate();
  }

  try {
    boost::program_options::notify(argument_variables);
  }
  catch(...){
    BOOST_LOG_TRIVIAL(fatal) << "Error during the command line argument parsing!";
    setErrorCode(1);
    terminate();
  }
  
  BOOST_LOG_TRIVIAL(info) << "Given parameters count: " << std::to_string(argc);
  for (auto entry : argument_variables)
  {
    BOOST_LOG_TRIVIAL(info) << entry.first << " " << entry.second.as<std::string>();
  }
}

bool Generator::str_replace(std::string& str, const std::string& from, const std::string& to)
{
  unsigned long start_pos = str.find(from);
  if (start_pos == std::string::npos) {
    BOOST_LOG_TRIVIAL(error) << "String replace failed, \"" + from + "\" is not found in the: \" " + str.substr(2) + "...\" string";
    return false;
  }
  str.replace(start_pos, from.length(), to);
  return true;
}

bool Generator::copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination)
{
  namespace fs = boost::filesystem;
  try
  {
    // Check and create the directory
    if (fs::exists(destination))
    {
      BOOST_LOG_TRIVIAL(info) << "Destination directory "
        << destination.string() << " already exists. Overriding...";
    }
    else
    {
      if (!fs::create_directory(destination))
      {
        BOOST_LOG_TRIVIAL(error) << "Unable to create destination directory "
          << destination.string();
        return false;
      }
    }
  }
  catch (fs::filesystem_error const & e)
  {
    BOOST_LOG_TRIVIAL(error) << e.what();
    return false;
  }

  // Check and copy the files
  for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file)
  {
    try {
      fs::path current(file->path());
      if (fs::is_directory(current))
      {
        if (!copyDir(current, destination / current.filename()))
        {
          return false;
        }
      }
      else
      {
        if (current.filename() == "property.h")
        {
          property_header_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "property.h found! Path: " << property_header_file_path;
        }

        if (current.filename() == "property.cpp")
        {
          property_cpp_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "property.cpp found! Path: " << property_cpp_file_path;
        }

        if(current.filename() == "monitor_name.cpp")
        {
          monitor_name_cpp_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "monitor_name.cpp found! Path: " << monitor_name_cpp_file_path;
          fs::copy_file(current, destination / (argument_variables["monitor-name"].as<std::string>() + ".cpp"), boost::filesystem::copy_option::overwrite_if_exists);
        }

        if(current.filename() == "CMakeLists.txt")
        {
          cmake_txt_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "CMakeLists.txt found! Path: " << cmake_txt_file_path;
        }

        if(current.filename() == "package.xml")
        {
          package_xml_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "CMakeLists.txt found! Path: " << package_xml_file_path;
        }

        if(current.filename() != "monitor_name.cpp")
          fs::copy_file(current, destination / current.filename(), boost::filesystem::copy_option::overwrite_if_exists);
      }
    }
    catch (fs::filesystem_error const & e)
    {
      BOOST_LOG_TRIVIAL(error) << e.what();
    }
  }
  return true;
}

void Generator::terminate()
{
  terminate(getErrorCode());
}

void Generator::terminate(int error_code)
{
  BOOST_LOG_TRIVIAL(info) << std::string("Terminating. Error value: ") + std::to_string(error_code);
  std::exit(error_code);
}