#pragma once

#ifdef __cpp_lib_optional
#include <optional>
#else
#include <boost/optional.hpp>
namespace std {
using boost::optional;  // To be replaced by std::optional in C++17
}
#endif
