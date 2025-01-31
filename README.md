# Map to Gazebo World

This is a naive python script which takes in a nav_msgs/OccupancyGrid style map described by a YAML file and converts it to a gazebo world. This is not the most 
efficient way to do so, but it is quick and simple to run.

### Usage

You can run the python file in the `src` directory using:

```
python map_to_world.py <PATH_TO_YAML_FILE> <DESIRED_NAME_OF_WORLD>.sdf
```

This will save the gazebo world as `DESIRED_NAME_OF_WORLD.sdf` which you can then use in a gazebo simulator.

### Requirements

This was developed using python version `3.11.1`

You will need pyyaml, numpy, opencv-python, and argparse to run this script.
