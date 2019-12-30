def get_plural_name(name):
    import inflect

    p = inflect.engine()

    return p.plural(name)


def add_feature_layer(layers, singular_feature_name, geometry_type, is_mandatory=False):
    layer_name = get_plural_name(singular_feature_name)

    layers[layer_name] = {
        "geometry": geometry_type,
        "properties": {},
        "meta": {"is_mandatory": is_mandatory},
    }

    return layers[layer_name]["properties"]


def add_relationship_layer(
    layers, singular_feature_name_1, singular_feature_name_2, layer_name=None, is_mandatory=False
):
    if not layer_name:
        plural_feature_name_1 = get_plural_name(singular_feature_name_1)
        plural_feature_name_2 = get_plural_name(singular_feature_name_2)
        layer_name = f"{plural_feature_name_1}_{plural_feature_name_2}"

    layers[layer_name] = {
        "geometry": "None",
        "properties": {f"{singular_feature_name_1}_id": "int64", f"{singular_feature_name_2}_id": "int64"},
        "meta": {"is_mandatory": is_mandatory},
    }

    return layers[layer_name]["properties"]


def get_autoware_vector_map_layers():
    layers = {}

    # Feature
    p = add_feature_layer(layers, "intersection_area", "3D Polygon")

    p = add_feature_layer(layers, "lane_section", "3D Polygon")

    p = add_feature_layer(layers, "lane", "3D LineString", is_mandatory=True)
    p["lane_section_id"] = "int64"
    p["width"] = "float"
    p["can_left_lane_change"] = "bool"
    p["can_right_lane_change"] = "bool"
    p["is_merge"] = "bool"
    p["is_diverge"] = "bool"
    p["is_intersection"] = "bool"
    p["is_left_turn"] = "bool"
    p["is_right_turn"] = "bool"

    p = add_feature_layer(layers, "lane_boundary", "3D LineString")
    p["width"] = "float"

    p = add_feature_layer(layers, "lane_divider_line", "3D LineString")
    p["width"] = "float"

    p = add_feature_layer(layers, "road_edge", "3D LineString")

    p = add_feature_layer(layers, "stop_line", "3D LineString", is_mandatory=True)
    p["is_reason_rule"] = "bool"
    p["is_reason_crosswalk"] = "bool"
    p["is_reason_traffic_light"] = "bool"
    p["is_reason_standby"] = "bool"
    p["is_reason_virtual"] = "bool"

    p = add_feature_layer(layers, "crosswalk", "3D LineString", is_mandatory=True)
    p["width"] = "float"

    p = add_feature_layer(layers, "traffic_light", "3D LineString", is_mandatory=True)
    p["code"] = "str"

    # Relationship
    p = add_relationship_layer(layers, "lane_section", "next_lane_section", "lane_section_connections")

    p = add_relationship_layer(layers, "lane", "next_lane", "lane_connections")

    p = add_relationship_layer(layers, "lane", "adjacent_lane", "adjacent_lanes")
    p["type"] = "str"

    p = add_relationship_layer(layers, "lane", "lane_boundary", "adjacent_lane_boundaries")
    p["type"] = "str"

    p = add_relationship_layer(layers, "lane", "stop_line")

    p = add_relationship_layer(layers, "lane", "crosswalk")

    p = add_relationship_layer(layers, "lane", "traffic_light")

    return layers

