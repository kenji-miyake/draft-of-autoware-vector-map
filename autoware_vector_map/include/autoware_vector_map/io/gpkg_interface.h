#pragma once

#include <memory>
#include <vector>

#include <autoware_vector_map/future/optional.h>

#include <autoware_vector_map/core.h>
#include <autoware_vector_map/traits/gpkg_contents.h>

class OGRLayer;
class GDALDataset;

namespace autoware_vector_map {
namespace io {

class GpkgInterface {
 public:
  explicit GpkgInterface(const char* gpkg_path);
  explicit GpkgInterface(const std::vector<uint8_t>& bin_data);

  void toFile(const char* gpkg_path);
  std::vector<uint8_t> toBinary();

  template <class T>
  std::optional<T> getFeatureById(const Id id);

  template <class T>
  std::vector<T> getFeaturesByIds(const std::vector<Id>& ids);

  template <class T, RelationSide S,
            class U = typename traits::gpkg_relationship<T>::template related_feature_t<S>>
  std::vector<U> getRelatedFeaturesById(const Id id,
                                        const std::function<bool(const T&)> predicate = nullptr);

  template <class T>
  std::vector<T> getAllFeatures();

  template <class T>
  std::vector<T> findFeaturesByRange(const Point3d& p, const double range);

 private:
  std::unique_ptr<GDALDataset> dataset_;

  template <class T>
  std::vector<T> getFeaturesBySql(const char* sql);

  template <class T>
  std::vector<T> getFeaturesByLayer(OGRLayer* layer);
};

}  // namespace io
}  // namespace autoware_vector_map

#include "gpkg_interface/gpkg_interface_impl.h"
