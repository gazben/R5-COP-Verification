#ifndef Generator_h__
#define Generator_h__

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <boost/log/trivial.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>

#include "SyntX/util/languages/ltl.h"
#include "SyntX/util/parser/parser.h"
#include "block_generator.h"
#include "connection_normalform_generator.h"
#include "ast_optimizer.h"

class Generator
{
public:
  Generator();
  ~Generator();

  void run(int argc, char* argv[]);
  void setExpressionInput(std::string expression_input);
  std::string getExpressionInput();
  void setMonitorSourcePath(std::string monitor_source_path);
  void setMonitorDestinationPath(std::string monitor_destination_path);
  int getErrorCode();
  void setErrorCode(int error_code);
  std::shared_ptr<base_rule::node> getRoot();
  void setRoot(std::shared_ptr<base_rule::node> root);
private:
  std::shared_ptr<base_rule::node> root;

  std::string property_cpp_file_path;
  std::string property_header_file_path;

  std::string property_cpp_file_content;
  std::string property_header_file_content;

  std::string expression_input;
  BlockGenerator block_generator;
  ConnectionNormalFormGenerator converter;

  std::string monitor_source_path;
  std::string monitor_destination_path;

  std::shared_ptr<base_rule::node> parseInput(std::string expression_input);
  void generateMonitor();
  bool str_replace(std::string& str, const std::string& from, const std::string& to);
  bool copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination);
  
  int error_code;
};
#endif
