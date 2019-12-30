from shapely.geometry import Point

from autoware_vector_map.map_api import MapApi


def create_lane_connection(lane_id, next_lane_id):
    return {"geometry": None, "properties": {"lane_id": lane_id, "next_lane_id": next_lane_id}}


def create_lane_connections(map_api: MapApi):
    lanes = map_api.get_all_features_as_gdf("lanes")

    lane_connections = []
    for base_lane in lanes.itertuples():
        touch_lanes = lanes[lanes.geometry.touches(base_lane.geometry)]
        for touch_lane in touch_lanes.itertuples():
            p_end = base_lane.geometry.coords[-1]
            p_start = touch_lane.geometry.coords[0]

            th_dist = 0.01
            if Point(p_end).distance(Point(p_start)) < th_dist:
                lane_connections.append(create_lane_connection(base_lane.id, touch_lane.id))

    return lane_connections
