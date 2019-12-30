#pragma once

#ifdef __cpp_lib_void_t
#include <type_traits>
#else
namespace std {
template <class...>
using void_t = void;  // To be replaced by std::void_t in C++17
}
#endif
