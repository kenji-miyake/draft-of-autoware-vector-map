from autoware_vector_map.layer import get_plural_name
from autoware_vector_map.map_api import MapApi
from autoware_vector_map import map_util


def create_intersect_relationship(base_feature_name, target_feature_name, base_feature, target_feature):
    return {
        "geometry": None,
        "properties": {f"{base_feature_name}_id": base_feature.id, f"{target_feature_name}_id": target_feature.id,},
    }


def create_intersect_relationships(map_api: MapApi, base_feature_name, target_feature_name, offset=None):
    base_features = map_api.get_all_features_as_gdf(get_plural_name(base_feature_name))
    target_features = map_api.get_all_features_as_gdf(get_plural_name(target_feature_name))

    intersect_relationships = []
    for target_feature in target_features.itertuples():
        target_geometry = target_feature.geometry
        if offset:
            assert target_geometry.geom_type == "LineString"
            target_geometry = map_util.parallel_offset_wrapper(target_geometry, offset, "left")

        intersect_base_features = base_features[base_features.geometry.intersects(target_geometry)]
        for base_feature in intersect_base_features.itertuples():
            intersect_relationships.append(
                create_intersect_relationship(base_feature_name, target_feature_name, base_feature, target_feature)
            )

    return intersect_relationships
