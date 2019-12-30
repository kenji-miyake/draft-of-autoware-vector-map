import argparse
import shutil
from pathlib import Path

import numpy as np
import open3d as o3d
from shapely.geometry import LinearRing, LineString, Point, Polygon

from autoware_vector_map.map_api import MapApi


def filter_points_by_rectangle(points, x_min, x_max, y_min, y_max):
    xs = points[:, 0]
    ys = points[:, 1]

    idx = np.where((x_min < xs) & (xs < x_max) & (y_min < ys) & (ys < y_max))

    return points[idx[0]]


def find_nearest_z_value(points, x, y, z, height_offset):
    margin = 5
    x_min, x_max = (x - margin, x + margin)
    y_min, y_max = (y - margin, y + margin)
    filtered_points = filter_points_by_rectangle(points, x_min, x_max, y_min, y_max)

    if not filtered_points.any():
        return None

    zs = filtered_points[:, 2]
    min_z = np.min(zs)
    search_margin = 1

    zs_in_range = zs[(min_z + height_offset - search_margin < zs) & (zs < min_z + height_offset + search_margin)]
    new_z = np.mean(zs_in_range)

    return new_z


def create_new_coord(points, coord, height_offset):
    x, y, z = (coord[0], coord[1], coord[2])
    new_z = find_nearest_z_value(points, x, y, z, height_offset)

    if new_z:
        return (x, y, new_z)
    else:
        return (x, y, z)


def create_new_geometry(points, geometry, height_offset):
    f_new_coord = lambda points, coord: create_new_coord(points, coord, height_offset)

    if geometry.geom_type == "Point":
        return Point(f_new_coord(points, geometry.coords[0]))

    x_min, y_min, x_max, y_max = geometry.bounds
    filtered_points = filter_points_by_rectangle(points, x_min, x_max, y_min, y_max)

    if geometry.geom_type == "LineString":
        return LineString([f_new_coord(filtered_points, c) for c in geometry.coords])

    if geometry.geom_type == "LinearRing":
        return LinearRing([f_new_coord(filtered_points, c) for c in geometry.coords])

    if geometry.geom_type == "Polygon":
        exterior = [f_new_coord(filtered_points, c) for c in geometry.exterior.coords]
        interiors = [[f_new_coord(filtered_points, c) for c in interior.coords] for interior in geometry.interiors]
        return Polygon(exterior, interiors)


def align_features(map_api: MapApi, table_name, points):
    gdf = map_api.get_all_features_as_gdf(table_name)

    if not gdf.geometry.any():
        return

    x_min, y_min, x_max, y_max = gdf.total_bounds
    filtered_points = filter_points_by_rectangle(points, x_min, x_max, y_min, y_max)

    if table_name in ["traffic_lights"]:
        height_offset = 5
    else:
        height_offset = 0

    for i, row in gdf.iterrows():
        gdf.loc[i, "geometry"] = create_new_geometry(filtered_points, row.geometry, height_offset)

    map_api.save_gdf(table_name, gdf)


def align_vector_map_to_pointcloud(gpkg_path, pointcloud_path):
    pcd = o3d.io.read_point_cloud(str(pointcloud_path))
    points = np.asarray(pcd.points)

    map_api = MapApi(gpkg_path)

    for table_name in map_api.get_table_names():
        align_features(map_api, table_name, points)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_gpkg_path", type=Path)
    parser.add_argument("input_pointcloud_path", type=Path)
    parser.add_argument("--output-gpkg-path", type=Path)
    ns = parser.parse_args()

    input_gpkg_path = ns.input_gpkg_path
    input_pointcloud_path = ns.input_pointcloud_path

    if ns.output_gpkg_path:
        output_gpkg_path = ns.output_gpkg_path
    else:
        output_gpkg_path = input_gpkg_path.parent / f"{input_gpkg_path.stem}_aligned{input_gpkg_path.suffix}"

    shutil.copy(input_gpkg_path, output_gpkg_path)

    align_vector_map_to_pointcloud(output_gpkg_path, input_pointcloud_path)


if __name__ == "__main__":
    main()
