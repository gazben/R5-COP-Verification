#ifndef Generator_h__
#define Generator_h__

/* GLOBAL INCLUDES */
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

/* LOCAL INCLUDES */
#include "syntx/ltl.h"
#include "block_generator.h"
#include "ast_optimizer.h"
/* INCLUDES END */

//0. filename (key) - entry.first 
//1. destination path - std::get<0>(entry.second)
//2. file content - std::get<1>(entry.second)
//3. copy file? (bool) - std::get<2>(entry.second)
using FilePathContentMap = std::map<std::string, std::tuple<std::string, std::string, bool>>;

class Generator
{
public:
  Generator();
  Generator(int argc, char* argv[]);
  ~Generator();

  void run();
  void setExpressionInput(std::string expression_input);
  std::string getExpressionInput();
  void setMonitorSourcePath(std::string monitor_source_path);
  void setMonitorDestinationPath(std::string monitor_destination_path);

  std::shared_ptr<base_rule::node> getRoot();
  void setRoot(std::shared_ptr<base_rule::node> root);
  void parseProgramArguments(int argc, char* argv[]);

  static boost::program_options::options_description arguments;
  static boost::program_options::variables_map argument_variables;

private:
  std::shared_ptr<base_rule::node> root;
  std::string monitor_source_path;
  std::string monitor_destination_path;
  FilePathContentMap gen_files;
  std::string expression_input;
  BlockGenerator block_generator;
  ConnectionNormalFormGenerator converter;

  std::shared_ptr<base_rule::node> parseInput(std::string expression_input);
  void generateMonitor();
  int string_replace_all(std::string& str, const std::string& from, const std::string& to);
  bool copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination);

  std::vector<std::string> program_arguments;
};
#endif
