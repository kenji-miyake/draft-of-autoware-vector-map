# draft-of-autoware-vector-map

Draft of Autoware Vector Map

## Description

This repository was created in order to show my idea to Autoware Maps and Routing WG's members.  
Please note that this is not completed enough yet and requires more development to be practical level.

## Features

- Easy Vector Map Creation using QGIS
- Automatic generation of relationships and features
  - Intersecting Stop Line / Crosswalk
  - Connected Lane / Adjacent Lane
  - Lane Section / Lane Boundary
- Z-values alignment(PointCloud is required)
- GIS compatible(because using GeoPackage)

  - Easy to develop utility scripts

- GeoPackage based Vector Map Loader
  - By ID, By position and range
  - [Simple C++ data structures](https://github.com/kenji-miyake/draft-of-autoware-vector-map/blob/master/autoware_vector_map/include/autoware_vector_map/core/vector_map/features.h) are available and easily customizable
- boost::geometry integration
- ROS integration
  - pub/sub of map message
  - RViz visualization

## Requirement

- Ubuntu 18.04
- ROS Melodic
- catkin build

## Installation

```sh
mkdir -p ~/avm_ws/src
cd ~/avm_ws/src
git clone https://github.com/kenji-miyake/draft-of-autoware-vector-map.git
cd ~/avm_ws
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
sudo apt -y install libfmt-dev # required until PR is merged to rosdep
catkin build
echo "source ~/avm_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you use scripts,

```sh
sudo apt -y install python3-venv python3.7 python3.7-dev
pip3 install poetry
roscd autoware_vector_map/scripts/autoware_vector_map/
CPLUS_INCLUDE_PATH=/usr/include/gdal C_INCLUDE_PATH=/usr/include/gdal poetry install
```

## Demo

1. Rviz visualization

```sh
roslaunch autoware_vector_map_examples odaiba-sample-tf.launch
roslaunch autoware_vector_map vector_map_loader.launch vector_map_path:=`rospack find autoware_vector_map_examples`/share/map/odaiba-sample/odaiba-sample_preprocess.gpkg
rviz -d `rospack find autoware_vector_map_examples`/share/map/odaiba-sample/odaiba-sample.rviz
```

2. Low level APIs
   See [autoware_vector_map_examples/src/example1/main.cpp](autoware_vector_map_examples/src/example1/main.cpp).

```sh
rosrun autoware_vector_map_examples example1
```

3. Vector Map APIs
   See [autoware_vector_map_examples/src/example2/main.cpp](autoware_vector_map_examples/src/example2/main.cpp).

```sh
rosrun autoware_vector_map_examples example2
```

## Vector Map Creation

### Sample

There are sample [map file](autoware_vector_map_examples/share/map/odaiba-sample/odaiba-sample.gpkg) and [project file](autoware_vector_map_examples/share/map/odaiba-sample/odaiba-sample.qgz).  
In the same directory, there is [odaiba-sample_preprocess.gpkg](autoware_vector_map_examples/share/map/odaiba-sample/odaiba-sample_preprocess.gpkg), which is a preprocessed file by script.  
Images are in [this document](autoware_vector_map_examples/doc/vector-map-creation.md).

`.qgz` files can be open with QGIS.  
Also, you can view `.gpkg` files using [DB Browser for SQLite](https://sqlitebrowser.org/) like this.

```sh
sudo apt -y install sqlitebrowser
xdg-open empty-vector-map.gpkg
```

### Create your own map

To create your own vector map, please install QGIS 3.10 or later from [official page](https://www.qgis.org/ja/site/forusers/download.html).

1. Create an empty Vector Map to make digitizing easy

```sh
roscd autoware_vector_map/scripts/autoware_vector_map/
poetry shell
python `rospack find autoware_vector_map`/scripts/autoware_vector_map/create_empty_vector_map.py
```

2. Open QGIS, drag-and-drop empty-vector-map.gpkg, and write features

- I will prepare tutorial videos later.

3. Run preprocess script

```sh
roscd autoware_vector_map/scripts/autoware_vector_map/
poetry shell
python `rospack find autoware_vector_map`/scripts/autoware_vector_map/vector_map_preprocessor.py {PATH_TO_YOUR_VECTOR_MAP}
```

## Contributing

I'd appreciate it if you could give me any feedback.

1. General comments about Vector Map Library for Autonomous Driving
   Please post to [this thread](https://discourse.ros.org/t/map-library-for-autoware-auto/11720).

2. Questions / Suggestions about this repository
   Please use [GitHub Issues](https://github.com/kenji-miyake/draft-of-autoware-vector-map/issues) or DM me on [Autoware Developers Slack](autoware-developer.slack.com).

3. Anything else
   Please DM me on [Autoware Developers Slack](autoware-developer.slack.com).
