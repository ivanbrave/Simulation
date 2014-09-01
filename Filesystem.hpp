#ifndef __FILESYSTEM_HPP__
#define __FILESYSTEM_HPP__

#include <boost/filesystem.hpp>
#include <string>

static inline bool existFile(const std::string & filename)
{
  return boost::filesystem::exists(filename);
}

#endif //__FILESYSTEM_HPP__
