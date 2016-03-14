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

  //add the wanted filenames for generation
  gen_files["gen_commands.h"] = std::make_tuple("", "");
  gen_files["gen_blocks.h"] = std::make_tuple("", "");
  gen_files["CMakeLists.txt"] = std::make_tuple("", "");
  gen_files["package.xml"] = std::make_tuple("", "");

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
  //copy the code skeleton to the 
  if (copyDir(boost::filesystem::path(monitor_source_path), boost::filesystem::path(monitor_destination_path)))
    BOOST_LOG_TRIVIAL(info) << monitor_source_path + " directory successfully copied to " + monitor_destination_path;
  else {
    BOOST_LOG_TRIVIAL(fatal) << "Error during source code directory copy. From: " 
      + monitor_source_path + " to " + monitor_destination_path;
    terminate();
  }

  //modify the content
  BOOST_LOG_TRIVIAL(info) << "Processing package.xml...";
  string_replace_all(std::get<1>(gen_files["package.xml"]), 
    "--monitor_name--",
    argument_variables["monitor-name"].as<std::string>()
  );

  BOOST_LOG_TRIVIAL(info) << "Processing CMakeLists.txt...";
  string_replace_all(std::get<1>(gen_files["CMakeLists.txt"]),
    "--monitor_name--",
    argument_variables["monitor-name"].as<std::string>()
    );

  BOOST_LOG_TRIVIAL(info) << "Processing gen_blocks.h...";
  string_replace_all(std::get<1>(gen_files["gen_blocks.h"]),
    "//--DECLARATIONS--",
    block_generator.getFunctionDeclarations()
  );
  string_replace_all(std::get<1>(gen_files["gen_blocks.h"]),
    "//--CONSTRUCTFUNCTIONS--",
    block_generator.getConstructFunctions()
  );
  string_replace_all(std::get<1>(gen_files["gen_blocks.h"]),
    "//--EVALFUNCTIONS--", 
    block_generator.getFunctions()
  );
  
  BOOST_LOG_TRIVIAL(info) << "Processing gen_commands.h...";
  string_replace_all(std::get<1>(gen_files["gen_commands.h"]), 
    "--true_command--", 
    argument_variables["true-command"].as<std::string>()
  );
  string_replace_all(std::get<1>(gen_files["gen_commands.h"]),
    "--false_command--", 
    argument_variables["false-command"].as<std::string>()
  );

  //write out the files
  for (auto& entry : gen_files) {
    std::ofstream file_out(std::get<0>(entry.second));
    if (file_out.is_open())
      BOOST_LOG_TRIVIAL(info) << std::get<0>(entry.second) + " file opened for writing";
    else
      BOOST_LOG_TRIVIAL(fatal) << "property.h file opening for writing failed";
    file_out.write(std::get<1>(entry.second).c_str(), std::get<1>(entry.second).size());
    file_out.close();
  }
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

int Generator::string_replace_all(std::string& str, const std::string& from, const std::string& to)
{
  int replace_count = 0;
  unsigned long start_pos = str.find(from);

  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.length(), to);
    pos += to.length();
    replace_count = replace_count + 1;
  }

  BOOST_LOG_TRIVIAL(info) << from << " replaced " + std::to_string(replace_count) + " times with " + to.substr(0, 20) + "...";
  return replace_count;
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
        //get the wanted files path and content
        for (auto& entry : gen_files) {
          if (current.filename() == std::get<0>(entry))
          {
            //path
            std::get<0>(entry.second) = std::string((destination / current.filename()).string());
            BOOST_LOG_TRIVIAL(info) << entry.first + " found! Path: " << std::get<0>(entry.second);
            //content
            std::ifstream file_in(std::get<0>(entry.second));
            if (file_in.is_open())
              BOOST_LOG_TRIVIAL(info) << std::get<0>(entry.second) + " is opened";
            else
              BOOST_LOG_TRIVIAL(fatal) << std::get<0>(entry.second) + " file opening failed";
            std::get<1>(entry.second) = std::string((std::istreambuf_iterator<char>(file_in)), std::istreambuf_iterator<char>());
          }
        }
        //copy the files
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
  #include <stdio.h>
  getchar();
  terminate(getErrorCode());
}

void Generator::terminate(int error_code)
{
  BOOST_LOG_TRIVIAL(info) << std::string("Terminating. Error value: ") + std::to_string(error_code);
  std::exit(error_code);
}