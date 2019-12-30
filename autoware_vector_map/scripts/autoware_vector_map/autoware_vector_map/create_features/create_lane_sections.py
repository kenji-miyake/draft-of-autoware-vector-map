from autoware_vector_map import map_util
from autoware_vector_map.map_api import MapApi


def create_lane_section(id, coordinates):
    return {"id": id, "geometry": {"type": "Polygon", "coordinates": coordinates}, "properties": {}}


def create_lane_sections(map_api: MapApi):
    lanes = map_api.get_all_features_as_gdf("lanes")
    all_lane_section_id_set = {lane.lane_section_id for lane in lanes.itertuples()}

    lane_sections = []
    for lane_section_id in all_lane_section_id_set:
        lanes_in_section = map_api.get_lanes_by_lane_section_id(lane_section_id)

        sorted_lanes = map_util.get_sorted_lanes_from_left_to_right(map_api, lanes_in_section)

        left_most_lane = sorted_lanes[0]
        right_most_lane = sorted_lanes[-1]

        # Alias
        left_half_width = left_most_lane.width / 2
        right_half_width = right_most_lane.width / 2

        # Shift geometry using width
        margin = 0.5
        left_geometry = map_util.parallel_offset_wrapper(left_most_lane.geometry, left_half_width + margin, "left")
        right_geometry = map_util.parallel_offset_wrapper(right_most_lane.geometry, right_half_width + margin, "right")

        start_additional_points = []
        end_additional_points = []
        for lane in sorted_lanes:
            start_additional_points.append(lane.geometry.coords[0])

        for lane in reversed(sorted_lanes):
            end_additional_points.append(lane.geometry.coords[-1])

        exterior = [
            [left_geometry.coords[0]]
            + start_additional_points
            + list(right_geometry.coords)
            + end_additional_points
            + list(reversed(left_geometry.coords))
        ]

        interiors = []

        coordinates = exterior + interiors

        lane_sections.append(create_lane_section(lane_section_id, coordinates))

    return lane_sections
