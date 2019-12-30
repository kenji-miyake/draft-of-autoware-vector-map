from autoware_vector_map.map_api import MapApi


def create_lane_section_connection(lane_section_id, next_lane_section_id):
    return {
        "geometry": None,
        "properties": {"lane_section_id": lane_section_id, "next_lane_section_id": next_lane_section_id},
    }


def create_lane_section_connections(map_api: MapApi):
    lanes = map_api.get_all_features_as_gdf("lanes")
    all_lane_section_id_set = {lane.lane_section_id for lane in lanes.itertuples()}

    lane_section_connections = []
    for lane_section_id in all_lane_section_id_set:
        lanes_in_section = list(lanes[lanes.lane_section_id == lane_section_id].itertuples())

        next_lane_section_id_set = set()
        for lane in lanes_in_section:
            for next_lane in map_api.get_next_lanes_by_id(lane.id):
                next_lane_section_id_set.add(next_lane.lane_section_id)

        for next_lane_section_id in next_lane_section_id_set:
            lane_section_connections.append(create_lane_section_connection(lane_section_id, next_lane_section_id))

    return lane_section_connections
