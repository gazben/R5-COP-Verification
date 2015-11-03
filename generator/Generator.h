#ifndef Generator_h__
#define Generator_h__

#include <string>
#include <sstream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <SyntX/util/languages/ltl.h>
#include <SyntX/util/parser/parser.h>

#include "BlockGenerator.h"
#include "ConnectionNormalFormGenerator.h"
#include "ast_optimizer.h"

class Generator
{
public:
  Generator();
  ~Generator();

  void setMonitorSourcePath(std::string monitor_source_path);
  void setMonitorDestinationPath(std::string monitor_destination_path);
  void generateMonitor(std::string expression_input);
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

  void generateMonitor();
  bool str_replace(std::string& str, const std::string& from, const std::string& to);
  bool copyDir(boost::filesystem::path const & source, boost::filesystem::path const & destination);
};
#endif
