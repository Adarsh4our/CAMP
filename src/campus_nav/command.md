# Commands to Test Campus Navigation

Yes: this workflow passed the development simulation checks, and you confirmed
that it ran correctly. Follow the order below using the existing ROS 2 Jazzy
workspace. Pedestrian and dog avoidance were verified at a requested spawn
distance of 6m; this does not guarantee every obstacle layout or campus route.

## 1. Build and test — Terminal 1

```bash
cd /home/adarsh4our/CAMP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --packages-select campus_nav --symlink-install
source install/setup.bash
python3 -m pytest src/campus_nav/test/test_odom_bridge.py -q
```

Expected: build succeeds and **9 tests pass**. Other workspace packages must
already be built.

## 2. Launch — Terminal 1

```bash
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py 2>&1 | tee /tmp/camp_test.log
```

Use your desktop's display if it differs from `:1`. Wait about **20 seconds
after the buggy appears** for sensor initialization. Keep Terminal 1 running.

## 3. Source every new terminal

Run once in Terminals 2, 3, and any additional monitoring terminals:

```bash
cd /home/adarsh4our/CAMP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

All terminals must use the same ROS domain and Gazebo partition if those
variables are set in your environment.

## 4. Check readiness — Terminal 2

```bash
ros2 lifecycle get /controller_server

ros2 param get /local_costmap/local_costmap obstacle_layer.raw_cloud.min_obstacle_height
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /controller_server FollowPath.ObstaclesCritic.consider_footprint
```

Expected: controller `active`; parameter values `0.45`, `1.2`, and `True`.

## 5. Watch status — Terminal 3

```bash
ros2 topic echo /campus/nav_status --field data
```

Optionally run this in Terminal 4:

```bash
ros2 topic echo /campus/safety_status --field data
```

## 6. Start driving — Terminal 2

```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'GOAL:Chemistry Block'}"
```

Expected: `PLANNING` then `NAVIGATING`. Let the buggy move for approximately
**8 seconds** along the initial straight road before spawning an obstacle.
Use this master command for the handoff tests; direct planner goals bypass
the master's mission state transitions.

## 7. Pedestrian test — Terminal 2

```bash
ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0
```

Expected: slow down, switch to Nav2, pass the pedestrian, rejoin the route,
and resume Pure Pursuit. Allow roughly a minute for completion. Confirm both
the visible pass and this message in Terminal 1:

```text
Nav2 avoidance complete — resuming Pure Pursuit.
```

Expected master transitions:

```text
NAVIGATING -> OBSTACLE_AVOIDANCE -> NAVIGATING
```

`CLEAR` alone does not prove the detour finished. Returning to `NAVIGATING`
alone also does not prove success: the master resumes the follower after a
Nav2 failure as well.

## 8. Dog test — fresh run

Press **Ctrl+C in Terminal 1**, wait for shutdown, and restart:

```bash
ros2 launch campus_nav full_simulation.launch.py 2>&1 | tee /tmp/camp_test.log
```

Wait for initialization, repeat the readiness checks and driving command
from steps 4 and 6, then spawn the dog after approximately 8 seconds of motion:

```bash
ros2 run campus_nav test_obstacle_spawner --type dog --distance 6.0
```

Expected: Nav2 passes the dog, completes the detour, and returns to normal
route following.

## 9. Speed-breaker test — fresh run

Restart the simulation as in step 8. Wait for initialization and repeat steps
4 and 6. After approximately 8 seconds of motion:

```bash
ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 6.0
```

Expected: `SLOWDOWN_BUMP`, Pure Pursuit stays enabled, and the buggy crosses
the bump without a Nav2 detour. If already traveling at the 0.8m/s bump limit,
there may be no visible speed reduction.

## 10. Two-leg mission command — optional, separate run

Use while the master is idle:

```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'MISSION:SAB C|Chemistry Block'}"
```

This requests pickup, a five-second boarding dwell, and drop-off. The verified
obstacle comparison uses the direct goal in step 6; it does not establish
completion of every two-leg mission.

## 11. Diagnostic commands

Run continuous monitors individually in an available terminal. Stop a monitor
with **Ctrl+C** before running another command there.

```bash
# LiDAR publication rate.
ros2 topic hz /cloud

# false during Nav2 control, true after the handoff back.
ros2 topic echo /campus/follower_enable

# Nav2 goal status: 4 = succeeded, 6 = aborted.
ros2 topic echo /follow_path/_action/status

# Commanded motion.
ros2 topic echo /cmd_vel

# Measured pose and velocity feedback.
ros2 topic echo /odom

# Vehicle position in map coordinates.
ros2 topic echo /campus/vehicle_pose

# Important messages from the saved launch log.
rg 'switching to Nav2|Dispatched|avoidance complete|avoidance ended|Failed to make progress|ERROR' \
  /tmp/camp_test.log
```

In RViz, inspect `/campus/avoidance_path` with a Path display and
`/local_costmap/costmap` with a Map display.

## 12. Remove obstacles and stop

```bash
ros2 run campus_nav test_obstacle_spawner --remove
```

Stop the simulation with **Ctrl+C in Terminal 1**. Use one obstacle per fresh
simulation for comparable tests. The launch command overwrites
`/tmp/camp_test.log`; save a copy before restarting if you want to retain it:

```bash
cp /tmp/camp_test.log /tmp/camp_pedestrian_test.log
```

The spawned pedestrian and dog are static models appearing while the buggy
moves. These commands do not test moving-agent prediction.
