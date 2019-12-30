from autoware_vector_map import map_util
from autoware_vector_map.map_api import MapApi


def create_lane_boundary(coordinates):
    return {"geometry": {"type": "LineString", "coordinates": coordinates}, "properties": {"width": 0.2}}


def create_lane_boundaries(map_api: MapApi):
    lane_sections = map_api.get_all_features_as_gdf("lane_sections")

    lane_boundaries = []
    for lane_section in lane_sections.itertuples():
        lanes_in_section = map_api.get_lanes_by_lane_section_id(lane_section.id)

        sorted_lanes = map_util.get_sorted_lanes_from_left_to_right(map_api, lanes_in_section)

        for i in range(len(sorted_lanes)):
            lane = sorted_lanes[i]

            if i == 0:
                left_coordinates = map_util.parallel_offset_wrapper(lane.geometry, lane.width / 2, "left")
                lane_boundaries.append(create_lane_boundary(left_coordinates))

            right_coordinates = map_util.parallel_offset_wrapper(lane.geometry, lane.width / 2, "right")
            lane_boundaries.append(create_lane_boundary(right_coordinates))

    return lane_boundaries
