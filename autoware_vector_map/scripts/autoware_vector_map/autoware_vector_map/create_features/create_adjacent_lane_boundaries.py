import numpy as np
import geopandas as gpd
from shapely.geometry import Point

from autoware_vector_map import geo_util
from autoware_vector_map.map_api import MapApi


def find_adjacent_lane_boundary(lane_boundaries: gpd.GeoDataFrame, lane, left_right):
    lane_coords = [Point(c) for c in lane.geometry.coords]
    lane_start_azimuth = geo_util.calc_azimuth(lane_coords[0], lane_coords[1])
    lane_end_azimuth = geo_util.calc_azimuth(lane_coords[-2], lane_coords[-1])

    # Calculate stats
    stats = []
    for lane_boundary in lane_boundaries.itertuples():
        coords = [Point(c) for c in lane_boundary.geometry.coords]

        start_lateral_offset = geo_util.calc_lateral_offset(lane_coords[0], coords[0], lane_start_azimuth)
        end_lateral_offset = geo_util.calc_lateral_offset(lane_coords[-1], coords[-1], lane_end_azimuth)

        if left_right == "right":
            start_lateral_offset = -start_lateral_offset
            end_lateral_offset = -end_lateral_offset

        start_azimuth = geo_util.calc_azimuth(coords[0], coords[1])
        end_azimuth = geo_util.calc_azimuth(coords[-2], coords[-1])

        start_azimuth_diff = abs(geo_util.normalize_radian(start_azimuth - lane_start_azimuth))
        end_azimuth_diff = abs(geo_util.normalize_radian(end_azimuth - lane_end_azimuth))

        stats.append(
            {
                "lane_boundary": lane_boundary,
                "start_lateral_offset": start_lateral_offset,
                "end_lateral_offset": end_lateral_offset,
                "start_azimuth_diff": start_azimuth_diff,
                "end_azimuth_diff": end_azimuth_diff,
            }
        )

    # Filter by conditions
    th_min_dist = -0.1
    th_max_dist = 3
    th_max_azimuth = np.deg2rad(30)
    candidate_stats = list(
        filter(
            lambda stat: (
                (th_min_dist <= stat["start_lateral_offset"] <= th_max_dist)
                and (th_min_dist <= stat["end_lateral_offset"] <= th_max_dist)
                and (stat["start_azimuth_diff"] <= th_max_azimuth)
                and (stat["end_azimuth_diff"] <= th_max_azimuth)
            ),
            stats,
        )
    )

    if not candidate_stats:
        return None

    # Sort by score
    sorted_stat = sorted(candidate_stats, key=lambda stat: stat["start_azimuth_diff"] + stat["end_azimuth_diff"])

    return sorted_stat[0]["lane_boundary"]


def create_adjacent_lane_boundary(lane_id, lane_boundary_id, left_right):
    return {
        "geometry": None,
        "properties": {"lane_id": lane_id, "lane_boundary_id": lane_boundary_id, "type": left_right},
    }


def create_adjacent_lane_boundaries(map_api: MapApi):
    lane_sections = map_api.get_all_features_as_gdf("lane_sections")
    lane_boundaries = map_api.get_all_features_as_gdf("lane_boundaries")

    adjacent_lane_boundaries = []
    for lane_section in lane_sections.itertuples():
        lanes_in_section = map_api.get_lanes_by_lane_section_id(lane_section.id)

        candidate_lane_boundaries = lane_boundaries[lane_boundaries.within(lane_section.geometry.buffer(3))]

        for lane in lanes_in_section:
            left_lane_boundary = find_adjacent_lane_boundary(candidate_lane_boundaries, lane, "left")
            right_lane_boundary = find_adjacent_lane_boundary(candidate_lane_boundaries, lane, "right")

            if left_lane_boundary:
                adjacent_lane_boundaries.append(create_adjacent_lane_boundary(lane.id, left_lane_boundary.id, "left"))
            if right_lane_boundary:
                adjacent_lane_boundaries.append(create_adjacent_lane_boundary(lane.id, right_lane_boundary.id, "right"))

    return adjacent_lane_boundaries
