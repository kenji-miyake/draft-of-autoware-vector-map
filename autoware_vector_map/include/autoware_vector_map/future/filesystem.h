#pragma once

#ifdef __cpp_lib_filesystem
#include <filesystem>
#else
#include <boost/filesystem.hpp>
namespace std {
namespace filesystem = boost::filesystem;  // To be replaced by std::filesystem in C++17
}
#endif
