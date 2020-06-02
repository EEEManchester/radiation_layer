# Radiation Costmap Plugin
## Overview

Ionising radiation can damage electronics and materials of robot platforms.  To increase the lifetime of robotic systems deployed in environments with high levels of ionising radiation, minimisation of exposure is critical.  Avoidance of radiation is also important to limit the amount of cross contamination which may occur from wheels, tracks or propeller wash picking up and transporting loose powdered radioactive material.

The existing ROS [navigation stack](http://wiki.ros.org/navigation) allows for occupancy grid representations of hazards (such as [obstacles](http://wiki.ros.org/costmap_2d/hydro/obstacles)) or objects of interest (such as [people](http://wiki.ros.org/social_navigation_layers)).  Path planners can use this occupancy grid for optimal planning whilst avoiding obstacles, based on total cost.

This plugin allows for additional layers of the stock ROS [costmap_2d](http://wiki.ros.org/costmap_2d) implementation to be added based on data from radiation instrumentation.  The cost associated with radiation levels can be scaled on an *ad hoc* basis.

# Installation

To build from this repo, clone the latest version from this repository into your catkin workspace and compile using

```
cd catkin_ws/src
git clone https://github.com/EEEManchester/radiation_layer.git
cd ..
catkin_make
```

This package currently has no dependancies on other packages, besides the stock ROS costmap_2d package from the navigation stack.

# How it works

The plugin layer subscribes to a stamped topic representing ionising radiation, and parses the location (using TF) and intensity of radiation to a cost on the costmap.  The radiation layer operates at the same resolution and size as the main costmap.

The average radiation value at each costmap cell is maintained, even when the costmap is resized.  Radiation value can be any units or scale (e.g. counts per second, Sv/hr), as the actual cost is calculated based on upper and lower thresholds.  Using thresholds allows for the robot to ignore areas of low radiation (e.g. background levels), and completely avoid areas with elevated radiation (e.g. high enough to immediately cause an electronic fault).

Cost in each cell ranges from 0 (free space - robot can travel with no penalty) to 254 (lethal obstacle - robot will certainly collide with an object), and therefore the average radiation value is scaled linearly between 0-254 based on:

```
cost = 254 * (radiation_value - lower_threshold) / (upper_threshold - lower_threshold)
```
The upper and lower thresholds are managed by the user, and can be changed at any time through use of [dynamically reconfigurable](http://wiki.ros.org/dynamic_reconfigure) parameters.

When the costmap is resized, all the averaged radiation values on a per grid cell basis are temporarily stored, then the costmap is repopulated once the costmap has been expanded.  In the event that the threshold values are altered (and therefore the cost 0-254 has changed), the entire plugin layer cost is recalculated and sent out to the rest of the costmap.

## Running an example
Once the package is installed, the example launch file can be run using:
```
roslaunch radiation_layer costmap_demonstrator.launch
```

This will start RVIZ for visualisation, a node to publish radiation data (mimicking a robot driving around a radioactive environment) and a costmap_2d node with the radiation_layer plugin.  The costmap_2d node also listens to a map produced by SLAM (provided by the [map_server](http://wiki.ros.org/map_server)).

In a separate terminal, type:
```
rosrun rqt_reconfigure rqt_reconfigure
```
The dynamically reconfigurable options can be changed by the user in real-time.  These parameters can be modified by other ROS nodes (allowing for autonomous re-weighting of cost).  Check out how to setup a dynamic reconfigure client [here](http://wiki.ros.org/dynamic_reconfigure/Tutorials).

In this example, the range of published radiation values are integer values between 0-254 already for clarity, with default lower threshold = 0, and upper_threshold = 200, i.e. values above 200 will be set at lethal obstacles.  The current radiation sensor is based on a single pixel camera, where only the green channel represents radiation intensity (as uint8 value).

### Dynamic Reconfigure Parameters

The plugin can be enabled and disabled using the ```enable``` functionality, the plugin layer will continue to process radiation data but no cost will be added to the costmap.  Behaviour of a robot with respect to radiation can be easily switched on and off using this enable functionality.

To accommodate the wide range of values and units of measuring radiation (for example 300 counts per second may be equivalent to 1 nSv/hr), the lower and upper thresholds consist of a double value (between -999 and 999) and a power multiplier.  For example to set an upper threshold of 1000 *units*, the value would be set to ```1``` and the mulitplier to ```kilo (3)```, i.e. one with three zeros.

The power modifier for thresholds ranges from:

- nano (E-9)
- micro (E-6)
- milli (E-3)
- unit (E0)
- kilo (E3)
- mega (E6)
- giga (E9)

# Loading the plugin for **costmap_2d** or **move_base**

Using the plugin for both costmap_2d or directly for path planning into [move_base](http://wiki.ros.org/move_base) is achieved through the use of .yaml files.  An example yaml file can be found in the package ```launch/yaml``` folder.

Costmap layers are accessed as per the order specified in the yaml file, for example,
```
plugins: 
    - {name: static_map,       type: "costmap_2d::StaticLayer"}
    - {name: obstacles,        type: "costmap_2d::VoxelLayer"}
    - {name: inflation,        type: "costmap_2d::InflationLayer"}
    - {name: radiation,        type: "radiation_layer_namespace::RadLayer"}
```
results in the ```inflation``` plugin layer only operating on the levels above itself (i.e. static_map and obstacles), not the ```radiation``` layer.  Order of plugins may affect the desired output and performance of the costmap.  To specific the topic name of the radiation sensor, use the snippet
```
radiation:
    radiation_topic: radiationTopic
```
with ```radiation:``` being the same name as in the previous plugins lists.

# Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/EEEManchester/radiation_layer/issues).