# node_edge_map_editor
## What's this
- Scripts for edit node edge maps.

## Environment
- ROS noetic (Ubuntu 20.04LTS)

## Dependencies
- [amsl_navigation](https://github.com/amslabtech/amsl_navigation.git)
   - Necessarily dependency is only [amsl_navigation_manager](https://github.com/amslabtech/amsl_navigation_manager.git). But recomend to install avobe metapackage.

## Install
- Python3 Modules
  - numpy math sys datetime threading subprocess yaml
  - If you are using pip3
    - ` pip3 install PyYAML` # Others are preinstalled in Python, if not please issue it.
- Clone this repository to your catkin workspace source directory.
  ```
      cd <YOUR_CATKIN_WS>/src
      git clone https://github.com/amslabtech/node_edge_map_editor.git
      catkin build node_edge_map_editor
  ```
  
## Setup
- Set arguments in `map_editor.launch`
   - `node_edge_map_file`: File name of node edge map.
   - `pcd_file`: File name of Point Cloud Map.
   - `rviz_config`: File name of rviz config.
   - `interval`: Interval time for publishing Point Cloud Map.

## Run
- `roslaunch node_edge_map_editor map_editor.launch`
   - At first, this scripts make backup file named `<node_edge_file_name>.bak`
   
## Functions
- In home mode
   - an: Add new node
   - dn: Delete node
   - mn: Move node
   - ae: Add edge
   - de: Delete edge
   - me: Move edge
   - s: Save the job
   - v: Print out current map state on your console.
   - r: Reload node edge map from yaml file
   - q: Quit this script.
   
  
