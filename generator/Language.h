#ifndef Language_h__
#define Language_h__

#include <boost/filesystem.hpp>
#include <boost/spirit/include/qi.hpp>

using namespace std;
namespace qi = boost::spirit::qi;

template <typename Iterator>
struct LTLgrammar : qi::grammar < Iterator, string > 
{
  LTLgrammar() : LTLgrammar::base_type{
  }

  qi::rule<Iterator, string> start;
};

#endif // Language_h__
