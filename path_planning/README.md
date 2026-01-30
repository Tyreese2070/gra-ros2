# path_planning
Generates the centreline of the track based on the selected event

## Usage
1. Source your virtual environment
2. Run the path planning node:
    ```bash
    ros2 run path_planning pathfinder.py
    ```
3. Visualise the path in Rviz
   ```bash
   rviz2
   ```
    
### Node Parameters
The following parameters are provided for `pathfinder.py`:

## Interface

| Node | Inputs | Outputs | Description |
|------|-------------|---------|---------|
| `pathfinder.py` | `/track_map` (`common_msgs/ConeArray`)<br>`/odom` (`nav_msgs/Odometry`) | `/path` (`nav_msgs/Path`) | Processes the track cone map and the vehicle pose to generate the centreline for different dynamic events |
