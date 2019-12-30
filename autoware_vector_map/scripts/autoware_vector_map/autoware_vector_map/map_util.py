from shapely.geometry import LineString

from autoware_vector_map.map_api import MapApi


# Create wrapper to avoid the problem written in https://github.com/Toblerity/Shapely/issues/284
def parallel_offset_wrapper(line_string: LineString, distance, side):
    offseted = line_string.parallel_offset(distance, side)

    if side == "right":
        offseted = LineString(reversed(offseted.coords))

    z = line_string.coords[0][2]
    coords_3d = [(c[0], c[1], z) for c in offseted.coords]

    return LineString(coords_3d)


def get_sorted_lanes_from_left_to_right(map_api: MapApi, lanes_in_section):
    left_most_lane = map_api.find_edge_lane(lanes_in_section, "left")

    sorted_lanes = [left_most_lane]
    base_lane = left_most_lane
    while True:
        right_lane = map_api.get_adjacent_lane_by_id(base_lane.id, "right")
        if not right_lane:
            break
        else:
            sorted_lanes.append(right_lane)
            base_lane = right_lane

    return sorted_lanes
