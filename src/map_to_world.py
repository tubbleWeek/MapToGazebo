import yaml
import numpy as np
import cv2
import argparse

def load_map(yaml_file):
    """Load map metadata from given YAML file."""
    with open(yaml_file, 'r') as f:
        return yaml.safe_load(f)

def merge_obstacles_rowwise(occupied_mask, resolution, origin):
    """Merge consecutive occupied cells row-wise to reduce obstacle count while preserving structure."""
    obstacles = []
    rows, cols = occupied_mask.shape

    for y in range(rows):
        x_start = None
        for x in range(cols):
            if occupied_mask[y, x]:  # Found an obstacle
                if x_start is None:
                    x_start = x  # Start new obstacle
            else:
                if x_start is not None:  # End of obstacle
                    width = (x - x_start) * resolution
                    world_x = origin[0] + (x_start + x - 1) / 2 * resolution
                    world_y = origin[1] + (rows - y - 1) * resolution  # Flip Y-axis

                    obstacles.append((world_x, world_y, width, resolution))
                    x_start = None  # Reset

        if x_start is not None:  # Row ends with an obstacle
            width = (cols - x_start) * resolution
            world_x = origin[0] + (x_start + cols - 1) / 2 * resolution
            world_y = origin[1] + (rows - y - 1) * resolution
            obstacles.append((world_x, world_y, width, resolution))

    return obstacles

def convert_occupancy_grid_to_sdf(map_data, output_sdf):
    """Convert a 2D occupancy grid into sdf world for Gazebo."""
    image_path = map_data['image']
    resolution = map_data['resolution']
    origin = map_data['origin']
    negate = map_data['negate']
    occupied_thresh = map_data['occupied_thresh']
    free_thresh = map_data['free_thresh']

    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise ValueError(f"Could not load image: {image_path}")

    # Convert image to occupancy grid
    image = 255 - image if negate else image
    occupancy_grid = (image / 255.0)

    # Thresholding
    occupied_mask = occupancy_grid < occupied_thresh  # Identify obstacles

    # Merge obstacles row-wise
    obstacles = merge_obstacles_rowwise(occupied_mask, resolution, origin)

    # Start creating SDF content
    sdf_content = f"""
    <sdf version='1.6'>
      <world name='optimized_map_world'>
        <include>
          <uri>model://sun</uri>
        </include>
        <include>
          <uri>model://ground_plane</uri>
        </include>
    """

    height = 0.5  # fixed height

    for i, (x, y, w, h) in enumerate(obstacles):
        sdf_content += f"""
        <model name='obstacle_{i}'>
          <static>true</static>
          <link name='link'>
            <collision name='collision'>
              <geometry>
                <box>
                  <size>{w} {h} {height}</size>
                </box>
              </geometry>
            </collision>
            <visual name='visual'>
              <geometry>
                <box>
                  <size>{w} {h} {height}</size>
                </box>
              </geometry>
            </visual>
          </link>
          <pose>{x} {y} {height/2} 0 0 0</pose>
        </model>
        """

    sdf_content += """
      </world>
    </sdf>
    """

    with open(output_sdf, 'w') as f:
        f.write(sdf_content)

    print(f"Optimized SDF world file saved as {output_sdf}, with {len(obstacles)} obstacles.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert 2D occupancy grid described by a YAML file to an optimized sdf world for Gazebo")
    parser.add_argument("yaml_file", help="Path to the YAML map file")
    parser.add_argument("output_sdf", help="Output SDF file path")

    args = parser.parse_args()
    map_data = load_map(args.yaml_file)
    convert_occupancy_grid_to_sdf(map_data, args.output_sdf)
