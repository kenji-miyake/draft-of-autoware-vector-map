import logging
from pathlib import Path

import fiona
import gdal
import geopandas as gpd
from osgeo import ogr
from shapely.geometry import mapping, shape

from autoware_vector_map.layer import get_autoware_vector_map_layers

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def load_features(input_path, layer_name):
    with fiona.open(input_path, "r", layer=layer_name) as f:
        features = list(f)
        crs = f.crs
        schema = f.schema

    for feature in features:
        feature["id"] = int(feature["id"])
        feature["properties"]["id"] = feature["id"]

    return features, crs, schema


def gdf_read_file_wrapper(input_path, layer_name):
    features, crs, schema = load_features(input_path, layer_name)

    columns = ["id"] + list(schema["properties"]) + ["geometry"]

    return gpd.GeoDataFrame.from_features(features, crs=crs, columns=columns)


class MapApi:
    def __init__(self, gpkg_path):
        self._gpkg_path = gpkg_path
        self._gdf_map = {}
        self._layers = get_autoware_vector_map_layers()

    def _fix_schema(self, table_name):
        features, crs, database_schema = load_features(self._gpkg_path, table_name)
        desired_schema = self.get_schema(table_name)

        is_diff_found = False

        # Add missing props
        for prop in desired_schema["properties"]:
            if not prop in database_schema["properties"]:
                is_diff_found = True
                logger.info(f"add `{prop}` to {table_name}")
                for feature in features:
                    feature["properties"][prop] = None

        # Remove invalid props
        for prop in database_schema["properties"]:
            if not prop in desired_schema["properties"]:
                is_diff_found = True
                logger.info(f"delete `{prop}` from {table_name}")
                for feature in features:
                    del feature["properties"][prop]

        if is_diff_found:
            self.save_fiona_objects(table_name, features, "w", crs)

    def _reload_table(self, table_name):
        self._gdf_map[table_name] = gdf_read_file_wrapper(self._gpkg_path, table_name)

    def _reload_all(self):
        for table_name in self.get_table_names():
            self._reload_table(table_name)

    def _check_non_empty(self, table_name):
        if self.get_all_features_as_gdf(table_name).empty:
            msg = f"table `{table_name}` is empty"
            logger.warn(msg)

    def _gpkg_exists(self):
        return Path(self._gpkg_path).exists()

    def _table_exists(self, table_name):
        if not self._gpkg_exists():
            return False
        else:
            return table_name in fiona.listlayers(self._gpkg_path)

    def is_mandatory(self, table_name):
        schema = self.get_schema(table_name)
        return schema["meta"]["is_mandatory"]

    def create_table(self, table_name, crs=None):
        schema = self.get_schema(table_name)

        if schema["geometry"] != "None" and crs is None:
            assert self._gpkg_exists()

            if self._table_exists(table_name):
                crs = self.get_crs(table_name)
            else:
                logger.info(f"`{table_name}`: using the same CRS as `lanes`")
                crs = self.get_crs("lanes")

        with fiona.open(self._gpkg_path, "w", driver="GPKG", crs=crs, schema=schema, layer=table_name) as f:
            pass

    def create_tables_if_not_exist(self, crs=None):
        for table_name in self.get_table_names():
            if not self._table_exists(table_name):
                logger.info(f"create new table `{table_name}`")
                self.create_table(table_name, crs)

    def fix_schemas(self):
        for table_name in self.get_table_names():
            self._fix_schema(table_name)

    def get_table_names(self):
        return self._layers.keys()

    def get_crs(self, table_name):
        if self._table_exists(table_name):
            _, crs, _ = load_features(self._gpkg_path, table_name)
            return crs
        else:
            return None

    def get_schema(self, table_name):
        return self._layers[table_name]

    def save_fiona_objects(self, table_name, objs, mode="w", crs=None):
        if not mode in ["w", "a"]:
            raise NotImplementedError("supported modes are ['w', 'a']")

        if mode == "w":
            if crs is None:
                crs = self.get_crs(table_name)
            fiona.remove(str(self._gpkg_path), layer=table_name)

        if not self._table_exists(table_name):
            self.create_table(table_name, crs)

        schema = self.get_schema(table_name)

        for obj in objs:
            if "id" in obj["properties"]:
                del obj["properties"]["id"]

        ### Don't use fiona's writerecords because it doesn't keep fid. ###
        ds = gdal.OpenEx(str(self._gpkg_path), gdal.OF_UPDATE | gdal.OF_VECTOR)
        layer = ds.GetLayerByName(table_name)

        # Add id if not exist
        for i, obj in enumerate(objs):
            if not "id" in obj:
                obj["id"] = i + 1

                # Get max id and increment in append mode
                if mode == "a":
                    gdf = self.get_all_features_as_gdf(table_name)
                    max_id = 0 if gdf.empty else max(gdf.id)
                    obj["id"] += max_id

        for obj in objs:
            feature = ogr.Feature(layer.GetLayerDefn())
            feature.SetFID(obj["id"])
            if obj["geometry"]:
                geometry = ogr.CreateGeometryFromWkt(shape(obj["geometry"]).wkt)
                feature.SetGeometry(geometry)

            for prop in schema["properties"]:
                feature.SetField(prop, obj["properties"][prop])

            ret = layer.CreateFeature(feature)

            if ret != 0:
                raise RuntimeError("failed to create feature")

        self._reload_table(table_name)

    def save_gdf(self, table_name, gdf):
        crs = self.get_crs(table_name)
        schema = self.get_schema(table_name)

        fiona_objects = []
        for feature in gdf.itertuples():
            props = {prop: getattr(feature, prop) for prop in schema["properties"].keys()}
            obj = {"id": feature.id, "geometry": mapping(feature.geometry), "properties": props}
            fiona_objects.append(obj)

        self.save_fiona_objects(table_name, fiona_objects, "w", crs)

        self._reload_table(table_name)

    def get_all_features_as_gdf(self, table_name) -> gpd.GeoDataFrame:
        if not table_name in self._gdf_map:
            self._reload_table(table_name)
        return self._gdf_map[table_name]

    def get_all_features(self, table_name):
        return list(self.get_all_features_as_gdf(table_name).itertuples())

    def get_feature_by_id(self, table_name, id):
        if isinstance(id, gpd.pd.Series):
            ids = id
        else:
            ids = [id]

        features = self.get_features_by_ids(table_name, ids)

        if not features:
            return None

        return features[0]

    def get_features_by_ids(self, table_name, ids):
        self._check_non_empty(table_name)

        if isinstance(ids, gpd.pd.Series) and ids.empty:
            return []

        gdf = self.get_all_features_as_gdf(table_name)
        row = gdf[gdf["id"].isin(ids)]

        if row.empty:
            return []

        return list(row.itertuples())

    def get_lanes_by_lane_section_id(self, lane_section_id):
        self._check_non_empty("lanes")

        gdf = self.get_all_features_as_gdf("lanes")
        row = gdf[(gdf["lane_section_id"] == lane_section_id)]

        return list(row.itertuples())

    def get_next_lanes_by_id(self, id):
        self._check_non_empty("lane_connections")

        gdf = self.get_all_features_as_gdf("lane_connections")
        row = gdf[(gdf["lane_id"] == id)]

        return self.get_features_by_ids("lanes", row["next_lane_id"])

    def get_prev_lanes_by_id(self, id):
        self._check_non_empty("lane_connections")

        gdf = self.get_all_features_as_gdf("lane_connections")
        row = gdf[(gdf["next_lane_id"] == id)]

        return self.get_features_by_ids("lanes", row["lane_id"])

    def get_adjacent_lane_by_id(self, id, left_right):
        self._check_non_empty("adjacent_lanes")

        gdf = self.get_all_features_as_gdf("adjacent_lanes")
        row = gdf[(gdf["lane_id"] == id) & (gdf["type"] == left_right)]

        return self.get_feature_by_id("lanes", row["adjacent_lane_id"])

    def find_edge_lane(self, lanes_in_section, left_right):
        edge_lane_id = lanes_in_section[0].id

        while True:
            adjacent_lane = self.get_adjacent_lane_by_id(edge_lane_id, left_right)
            if not adjacent_lane:
                break
            else:
                edge_lane_id = adjacent_lane.id

        return self.get_feature_by_id("lanes", edge_lane_id)
