import argparse
from pathlib import Path

from autoware_vector_map.map_api import MapApi


def create_empty_vector_map(gpkg_path, crs, only_mandatory):
    map_api = MapApi(gpkg_path)

    for table_name in map_api.get_table_names():
        if only_mandatory:
            if not map_api.is_mandatory(table_name):
                continue
        map_api.create_table(table_name, crs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-path", type=Path, default="./empty-vector-map.gpkg")
    parser.add_argument("--crs", type=str, default="WGS84")
    parser.add_argument("--only-mandatory", type=bool, default=True)
    ns = parser.parse_args()

    output_path = ns.output_path

    if output_path.exists():
        msg = f"{output_path.absolute()} already exists. Please remove it first."
        raise FileExistsError(msg)

    create_empty_vector_map(output_path, ns.crs, ns.only_mandatory)


if __name__ == "__main__":
    main()
