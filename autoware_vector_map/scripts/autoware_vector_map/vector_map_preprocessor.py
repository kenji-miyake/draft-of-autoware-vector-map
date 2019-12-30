import argparse
import logging
import shutil
from pathlib import Path

import autoware_vector_map.create_features as cf
from autoware_vector_map.map_api import MapApi


logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def vector_map_preprocessor(gpkg_path):
    map_api = MapApi(gpkg_path)
    map_api.create_tables_if_not_exist()
    map_api.fix_schemas()

    map_api.save_fiona_objects("lane_connections", cf.create_lane_connections(map_api))

    map_api.save_fiona_objects(
        "lanes_stop_lines", cf.create_intersect_relationships(map_api, "lane", "stop_line", offset=0.5)
    )
    map_api.save_fiona_objects("lanes_crosswalks", cf.create_intersect_relationships(map_api, "lane", "crosswalk"))

    map_api.save_fiona_objects("adjacent_lanes", cf.create_adjacent_lanes(map_api))

    map_api.save_fiona_objects("lane_sections", cf.create_lane_sections(map_api))
    map_api.save_fiona_objects("lane_section_connections", cf.create_lane_section_connections(map_api))

    map_api.save_fiona_objects("lane_boundaries", cf.create_lane_boundaries(map_api))
    map_api.save_fiona_objects("adjacent_lane_boundaries", cf.create_adjacent_lane_boundaries(map_api))


def main():
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.INFO)

    logging.basicConfig(
        handlers=[stream_handler], format="%(name)s %(levelname)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S",
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("input_path", type=Path)
    parser.add_argument("--output-path", type=Path)
    ns = parser.parse_args()

    input_path = ns.input_path

    if ns.output_path:
        output_path = ns.output_path
    else:
        output_path = input_path.parent / f"{input_path.stem}_preprocess{input_path.suffix}"

    shutil.copy(input_path, output_path)

    # Remove temporary files
    tmp_files = [Path(str(output_path) + "-shm"), Path(str(output_path) + "-wal")]
    for tmp_file in tmp_files:
        if tmp_file.exists():
            tmp_file.unlink()

    vector_map_preprocessor(output_path)


if __name__ == "__main__":
    main()
