from shapely.geometry import Point

from autoware_vector_map import geo_util
from autoware_vector_map.map_api import MapApi


def find_adjacent_lane(lanes_in_section, base_lane, left_right):
    base_coords = [Point(c) for c in base_lane.geometry.coords]
    start_azimuth = geo_util.calc_azimuth(base_coords[0], base_coords[1])
    end_azimuth = geo_util.calc_azimuth(base_coords[-2], base_coords[-1])

    # Calculate stats
    stats = []
    for target_lane in lanes_in_section:
        if target_lane.id == base_lane.id:
            continue

        target_coords = [Point(c) for c in target_lane.geometry.coords]

        start_lateral_offset = geo_util.calc_lateral_offset(base_coords[0], target_coords[0], start_azimuth)
        end_lateral_offset = geo_util.calc_lateral_offset(base_coords[-1], target_coords[-1], end_azimuth)

        if left_right == "right":
            start_lateral_offset = -start_lateral_offset
            end_lateral_offset = -end_lateral_offset

        stats.append(
            {
                "lane": target_lane,
                "start_lateral_offset": start_lateral_offset,
                "end_lateral_offset": end_lateral_offset,
            }
        )

    # Filter by conditions
    th_min_dist = -0.1
    th_max_dist = 20
    candidate_stats = list(
        filter(
            lambda stat: (
                (th_min_dist <= stat["start_lateral_offset"] <= th_max_dist)
                and (th_min_dist <= stat["end_lateral_offset"] <= th_max_dist)
            ),
            stats,
        )
    )

    if not candidate_stats:
        return None

    # Sort by score
    sorted_stat = sorted(candidate_stats, key=lambda stat: stat["start_lateral_offset"])

    return sorted_stat[0]["lane"]


def create_adjacent_lane(lane_id, adjacent_lane_id, left_right):
    return {
        "geometry": None,
        "properties": {"lane_id": lane_id, "adjacent_lane_id": adjacent_lane_id, "type": left_right},
    }


def create_adjacent_lanes(map_api: MapApi):
    lanes = map_api.get_all_features_as_gdf("lanes")
    all_lane_section_id_set = {lane.lane_section_id for lane in lanes.itertuples()}

    adjacent_lanes = []
    for lane_section_id in all_lane_section_id_set:
        lanes_in_section = map_api.get_lanes_by_lane_section_id(lane_section_id)

        if len(lanes_in_section) < 2:
            continue

        for lane in lanes_in_section:
            left_lane = find_adjacent_lane(lanes_in_section, lane, "left")
            right_lane = find_adjacent_lane(lanes_in_section, lane, "right")

            if left_lane:
                adjacent_lanes.append(create_adjacent_lane(lane.id, left_lane.id, "left"))
            if right_lane:
                adjacent_lanes.append(create_adjacent_lane(lane.id, right_lane.id, "right"))

    return adjacent_lanes
