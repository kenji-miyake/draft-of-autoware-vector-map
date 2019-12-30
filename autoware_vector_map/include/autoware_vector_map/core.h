#pragma once

#include <memory>

namespace autoware_vector_map {

template <class T>
using Ptr = std::shared_ptr<T>;

template <class T>
using ConstPtr = std::shared_ptr<const T>;

using Id = int64_t;
constexpr Id invalid_id = 0;

}  // namespace autoware_vector_map

#include "core/geometries.h"

#include "core/vector_map/attributes.h"
#include "core/vector_map/features.h"
#include "core/vector_map/relationships.h"
