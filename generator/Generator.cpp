#include "generator.h"

Generator::Generator() :error_code(0)
{
}

Generator::~Generator()
{
}

void Generator::run(int argc, char* argv[])
{
  namespace po = boost::program_options;

  BOOST_LOG_TRIVIAL(info) << "ROS runtime monitor generator tool.";
  BOOST_LOG_TRIVIAL(info) << "Made by Bence Gazder.";

  try {
    /*
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("compression", po::value<double>(), "set compression level")
      ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
    }

    if (vm.count("compression")) {
      std::cout << "Compression level was set to "
        << vm["compression"].as<double>() << ".\n";
    }
    else {
      std::cout << "Compression level was not set.\n";
    }
    */
    std::string monitor_source_path = "D:\\Projects\\R5-COP-Verification\\monitor";
    std::string monitor_destination_path = "D:\\Projects\\R5-COP-Verification\\generated";

    //Example: std::string input = "G (((8 | 9) ^ 4) U (1 & 2))\n";
    std::string input = "G(1 => (2 U 3))";

    setMonitorDestinationPath(monitor_destination_path);
    setMonitorSourcePath(monitor_source_path);
    setExpressionInput(input);
    setRoot(parseInput(getExpressionInput()));

    AstOptimizer::optimizeAst(getRoot());   //remove the unnecessary parts of the AST
    block_generator.setAstRootNode(converter.convertToConnectionNormalForm(root));
    block_generator.createBlocks();
    generateMonitor();
    BOOST_LOG_TRIVIAL(info) << "Generation completed!";
    BOOST_LOG_TRIVIAL(info) << "Press enter to quit.";
    getchar();
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
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp is NOT found!";

  if (property_header_file_path.empty())
    BOOST_LOG_TRIVIAL(fatal) << "Property.h is NOT found!";

  std::ifstream propertyCppFile_in(property_cpp_file_path);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.cpp is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp opening failed";
  property_cpp_file_content = std::string((std::istreambuf_iterator<char>(propertyCppFile_in)), std::istreambuf_iterator<char>());

  std::ifstream peopertyHeaderFile(property_header_file_path);
  if (propertyCppFile_in.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.h is opened";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.h opening failed";
  property_header_file_content = std::string((std::istreambuf_iterator<char>(peopertyHeaderFile)), std::istreambuf_iterator<char>());

  if (str_replace(property_header_file_content, "//--DECLARATIONS--", block_generator.getFunctionDeclarations()))
    BOOST_LOG_TRIVIAL(info) << "Function declarations written to the Property.h file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Function declaration writing to the Property.h file failed!";

  if (str_replace(property_cpp_file_content, "//--CONSTRUCTFUNCTIONS--", block_generator.getConstructFunctions()))
    BOOST_LOG_TRIVIAL(info) << "Construct functions written to the Property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Construct functions writing to the Property.cpp file failed!";

  if (str_replace(property_cpp_file_content, "//--EVALFUNCTIONS--", block_generator.getFunctions()))
    BOOST_LOG_TRIVIAL(info) << "EvalFunctions written to the Property.cpp file";
  else
    BOOST_LOG_TRIVIAL(fatal) << "EvalFunction writing to the Property.cpp failed";

  std::ofstream propertyCppFile_out(property_cpp_file_path);
  if (propertyCppFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.cpp file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.cpp fille opening for writing failed";
  propertyCppFile_out.write(property_cpp_file_content.c_str(), property_cpp_file_content.size());
  propertyCppFile_out.close();

  std::ofstream propertyHeaderFile_out(property_header_file_path);
  if (propertyHeaderFile_out.is_open())
    BOOST_LOG_TRIVIAL(info) << "Property.h file opened for writing";
  else
    BOOST_LOG_TRIVIAL(fatal) << "Property.h file opening for writing failed";
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
        if (current.filename() == "Property.h")
        {
          property_header_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "Property.h found! Path: " << property_cpp_file_path;
        }

        if (current.filename() == "Property.cpp")
        {
          property_cpp_file_path = std::string((destination / current.filename()).string());
          BOOST_LOG_TRIVIAL(info) << "Property.cpp found! Path: " << property_cpp_file_path;
        }
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