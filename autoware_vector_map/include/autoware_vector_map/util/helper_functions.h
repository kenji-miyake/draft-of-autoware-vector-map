#pragma once

#include <algorithm>
#include <functional>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware_vector_map {
namespace util {

using std::back_inserter;
using std::begin;
using std::end;

// To be replaced by std::invoke_result in C++17
template <class T_container, class Predicate, class Input = typename T_container::value_type,
          class Output = typename std::result_of<Predicate(Input)>::type>
std::vector<Output> map(const T_container& c, const Predicate& f) {
  std::vector<Output> r;
  r.reserve(c.size());
  std::transform(begin(c), end(c), back_inserter(r), f);
  return r;
}

template <class T_container, class Predicate, class Input = typename T_container::value_type,
          class Output = Input>
std::vector<Output> filter(const T_container& c, const Predicate& f) {
  std::vector<Output> r;
  r.reserve(c.size());
  std::copy_if(begin(c), end(c), back_inserter(r), f);
  return r;
}

template <class T_container, class Input = typename T_container::value_type, class Output = Input>
std::vector<Output> sorted(const T_container& c) {
  std::vector<Output> r(begin(c), end(c));
  std::sort(r.begin(), r.end());
  return r;
}

template <class T_container, class Input = typename T_container::value_type, class Output = Input>
std::vector<Output> unique(const T_container& c) {
  std::unordered_set<Output> s(begin(c), end(c));
  std::vector<Output> r(begin(s), end(s));
  return r;
}

template <class T_container, class T = typename T_container::value_type, class Predicate,
          std::enable_if_t<std::is_convertible<Predicate, std::function<bool(const T&)>>::value,
                           std::nullptr_t> = nullptr>
bool contains(const T_container& c, const Predicate f) {
  for (const auto& v : c) {
    if (f(v)) {
      return true;
    }
  }
  return false;
}

template <class T_container>
bool contains(const T_container& c, const typename T_container::value_type& value) {
  for (const auto& v : c) {
    if (v == value) {
      return true;
    }
  }
  return false;
}

template <class T_container1, class T_container2 = std::vector<typename T_container1::value_type>>
bool contains_any(const T_container1& c, const T_container2& values) {
  for (const auto& value : values) {
    if (contains(c, value)) {
      return true;
    }
  }
  return false;
}

template <class T_container1, class T_container2 = std::vector<typename T_container1::value_type>>
bool contains_all(const T_container1& c, const T_container2& values) {
  for (const auto& value : values) {
    if (!contains(c, value)) {
      return false;
    }
  }
  return true;
}

template <class T_container1, class T_container2 = std::vector<typename T_container1::value_type>>
bool equal(const T_container1& c1, const T_container2& c2) {
  if (c1.size() != c2.size()) {
    return false;
  }

  for (size_t i = 0; i < c1.size(); ++i) {
    if (c1.at(i) != c2.at(i)) {
      return false;
    }
  }

  return true;
}

template <class T>
std::vector<Id> to_ids(const std::vector<T>& ids) {
  return map(ids, [](const T& f) { return f.id; });
}

template <class T_container>
size_t argmin(const T_container& c) {
  const auto itr = std::min_element(begin(c), end(c));
  return std::distance(begin(c), itr);
}

template <class T_container>
size_t argmax(const T_container& c) {
  const auto itr = std::max_element(begin(c), end(c));
  return std::distance(begin(c), itr);
}

}  // namespace util
}  // namespace autoware_vector_map
