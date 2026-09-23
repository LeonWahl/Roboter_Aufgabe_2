# Volksbot base stack

Everything under a SPLAM solution, and none of the solution: motor driver,
URDF, IMU, EKF, lidar, slam_toolbox and Nav2, in layers you can switch on and
off one at a time. Bring it up, drive the robot with a joystick or send it a
goal in RViz, and build your frontier exploration on top of what it publishes.

## Die Aufgabe

- "Simultaneous Planning Localization and Mapping"
- Idee: Baue selbstständig inkrementell eine Karte der Umgebung auf
- Fahre Posen an, an denen die Information über neue Gebiete vermutlich am
  größten ist
- "Kanten" zwischen bekannten und unbekannten Regionen
- Wenn keine neuen Informationen reinkommen (oder eine vorgegebene Zeit
  abgelaufen ist), speichere die Karte und stoppe

This workspace gives you the map, the pose and the ability to drive to a pose.
The frontier search, the decision of where to go next, and the save-and-stop at
the end are yours to write.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`src/` carries four vendored packages (see `vendor.md` for the upstreams and
commits). `epos2_motor_controller` needs `libftdi` and the EPOS2 board, so it
only builds on the robot.

## Run

```bash
# the whole thing. Nothing drives itself: Nav2 goes where it is told, and
# nothing here is telling it. Send it somewhere with RViz's "Nav2 Goal" tool
ros2 launch volksbot_bringup bringup.launch.py rviz:=true

# drivers and a map, nothing above them. Drive it by hand and watch /map grow
ros2 launch volksbot_bringup bringup.launch.py nav2:=false use_joystick:=true
```

### Components

Every component is on by default. Switch off the ones you do not want:

| argument | what it starts | default |
|---|---|---|
| `driver` | robot_state_publisher + the motor driver | true |
| `localization` | Phidgets IMU + EKF, `odom -> base_footprint` | true |
| `lidar` | `sick`, `rplidar` or `none` | sick |
| `slam` | slam_toolbox: `/map` and `map -> odom` | true |
| `nav2` | the six Nav2 servers + lifecycle manager | true |

```bash
# Nav2 alone: restart it without restarting the motor driver. Needs /scan,
# /odom, /map and TF from a second bringup with nav2:=false.
ros2 launch volksbot_bringup bringup.launch.py \
     driver:=false localization:=false lidar:=none slam:=false

# any single component, off
ros2 launch volksbot_bringup bringup.launch.py slam:=false
ros2 launch volksbot_bringup bringup.launch.py lidar:=none
```

`ros2 launch volksbot_bringup bringup.launch.py --show-args` lists every
argument with its description.

## What you get to build on

| | |
|---|---|
| `/scan` | lidar |
| `/odom` | wheel odometry |
| `/map` | the map, from slam_toolbox |
| `map -> odom -> base_footprint` | TF, from slam_toolbox and the EKF |
| `/global_costmap/costmap` | Nav2, plus `/global_costmap/costmap_updates` |
| `/navigate_to_pose` | Nav2 action server: send it a `PoseStamped` |
| `/cmd_vel` | what the motors listen to, in the end |

Two things worth knowing before you write the explorer:

- **`track_unknown_space: true`** in `config/nav2_params.yaml` is what makes
  frontiers exist at all. A frontier is a cell where known free space touches
  `NO_INFORMATION`; with that flag false, everything unseen reads as free and
  there is nothing to find.
- **`always_send_full_costmap: true`** in the same file. Nav2 publishes the
  costmap `transient_local`, but a subscriber created with default (volatile)
  durability gets no latched history — only new publications. A costmap client
  that starts after Nav2 did will otherwise wait forever for its first message.
  Either keep this flag or subscribe `transient_local`.

Saving the map when you are done:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
```

## Tuning

All of it is in `src/volksbot_bringup/config/`, and the files are commented
with why each value is what it is.

| file | what it controls |
|---|---|
| `nav2_params.yaml` | the whole Nav2 stack |
| `slam_toolbox/online_async.yaml` | mapping, and the `map -> odom` timing |
| `ekf.yaml` | what the EKF fuses, and how |
| `bt/` | behavior trees, forked from the Nav2 defaults so recovery never reverses — the 270° SICK cannot see behind the robot |

The laser and IMU offsets in `bringup.launch.py` are **placeholders**. Measure
them on the robot in front of you before trusting where the costmaps put an
obstacle.
