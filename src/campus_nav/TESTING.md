# Testing the Hybrid Navigation Workflow

This guide covers normal driving, an obstacle appearing ahead of the moving
buggy, Nav2 passing and rejoining the route, and speed-breaker behavior. The
user confirmed that the supplied full-simulation procedure ran correctly on
2026-10-16. Detailed development results are in the
[validation record](test/OBSTACLE_AVOIDANCE_VALIDATION.md).

## 1. Build and run the regression tests — Terminal 1

```bash
cd /home/adarsh4our/CAMP
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon build --packages-select campus_nav --symlink-install
source install/setup.bash
python3 -m pytest src/campus_nav/test/test_odom_bridge.py -q
```

Expected: the package builds and all **9 tests pass**. These tests check that
odometry turn-rate feedback agrees with the change in navigation heading,
including left/right turns and angle wraparound. Other workspace packages
must already be built.

## 2. Start the full simulation — Terminal 1

```bash
export DISPLAY=:1
ros2 launch campus_nav full_simulation.launch.py 2>&1 | tee /tmp/camp_test.log
```

Use the display appropriate to your desktop if it differs from `:1`.
Wait about **20 seconds after the buggy appears** for sensor initialization.
The default spawn is LOC. Keep this terminal running.

## 3. Prepare each additional terminal

Run this once in every new terminal:

```bash
cd /home/adarsh4our/CAMP
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Use the same ROS domain and Gazebo partition in all terminals if your
environment sets them.

## 4. Check readiness — Terminal 2

```bash
ros2 lifecycle get /controller_server
```

Expected: `active`. Then verify the corrected configuration:

```bash
ros2 param get /local_costmap/local_costmap obstacle_layer.raw_cloud.min_obstacle_height
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /controller_server FollowPath.ObstaclesCritic.consider_footprint
```

Expected values: `0.45`, `1.2`, and `True`, respectively.

## 5. Monitor navigation — Terminal 3

```bash
ros2 topic echo /campus/nav_status --field data
```

Leave this running. Optionally monitor the safety classifier in Terminal 4:

```bash
ros2 topic echo /campus/safety_status --field data
```

## 6. Verify normal driving — Terminal 2

```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'GOAL:Chemistry Block'}"
```

Expected: `PLANNING` followed by `NAVIGATING`, and the buggy following its
route. Let it drive for approximately **8 seconds** on the initial straight
road before spawning an obstacle. Baseline driving in the development trials
commanded 0.8m/s; the configured cruise ceiling is not a guaranteed speed.

Use `/campus/master_command` for these tests. Direct planner inputs such as
`/campus/goal_stop` or RViz `/goal_pose` bypass the master's mission state
transitions, while `/campus/mission` only logs a migration instruction.

## 7. Test a pedestrian appearing while driving — Terminal 2

```bash
ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0
```

Expected sequence:

1. The buggy detects the obstacle and slows down.
2. The master changes from `NAVIGATING` to `OBSTACLE_AVOIDANCE`.
3. Pure Pursuit is disabled and Nav2 drives the lateral detour.
4. The buggy passes the pedestrian and returns toward its original route.
5. Terminal 1 reports `Nav2 avoidance complete — resuming Pure Pursuit.`
6. The master returns to `NAVIGATING`, and normal route following continues.

Allow roughly a minute for the complete maneuver. Confirm both the visible
pass and successful action completion. `CLEAR` can appear as soon as the
obstacle leaves the forward detection corridor, before the maneuver finishes.
The master can also return to Pure Pursuit after a Nav2 failure, so the state
transition alone is insufficient.

## 8. Repeat with a dog

Press **Ctrl+C in Terminal 1**, wait for shutdown, then restart with the launch
command in step 2. After initialization, repeat steps 4 and 6. Once the buggy
has been moving for approximately 8 seconds, run:

```bash
ros2 run campus_nav test_obstacle_spawner --type dog --distance 6.0
```

Expected: the same Nav2 handoff, successful pass, and return to the original
route. The low dog model must remain detectable after road-surface filtering.

## 9. Repeat with a speed breaker

Start another fresh simulation, wait for initialization, and repeat the goal
and initial driving steps. Then run:

```bash
ros2 run campus_nav test_obstacle_spawner --type speed_breaker --distance 6.0
```

Expected: the safety classifier reports `SLOWDOWN_BUMP`, Pure Pursuit remains
enabled, and the buggy crosses the bump and continues. It should not enter
`OBSTACLE_AVOIDANCE`. If it is already driving at the 0.8m/s bump limit, there
may be no visible speed reduction.

## 10. Additional mission interface

The master also accepts a two-leg pickup/boarding/drop-off request:

```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'MISSION:SAB C|Chemistry Block'}"
```

Dispatch this separately while the master is idle. This exercises a different
mission sequence from the direct-goal obstacle comparison above; the obstacle
fix does not establish completion of every campus route or multi-leg mission.

## Diagnostics

Run continuous monitors individually in an available terminal; use Ctrl+C to
stop a monitor without stopping the simulation.

```bash
# Confirm LiDAR messages are arriving.
ros2 topic hz /cloud

# Observe the controller handoff: false for Nav2, true after return.
ros2 topic echo /campus/follower_enable

# Inspect Nav2 action status. Status 4 means success; 6 means aborted.
ros2 topic echo /follow_path/_action/status

# Inspect velocity commands and measured bridge feedback.
ros2 topic echo /cmd_vel
ros2 topic echo /odom

# Inspect navigation-frame vehicle position.
ros2 topic echo /campus/vehicle_pose

# Extract the handoff, completion, and failure messages from the launch log.
rg 'switching to Nav2|Dispatched|avoidance complete|avoidance ended|Failed to make progress|ERROR' \
  /tmp/camp_test.log
```

In RViz, add a Path display for `/campus/avoidance_path` to inspect the detour,
and a Map display for `/local_costmap/costmap` to inspect Nav2's obstacles.
On the tested clear road, road-surface returns should not cover the driving
area with occupied cells. The spawned pedestrian or dog should appear.

| Symptom | Check |
|---|---|
| Nav2 is not active | Read Terminal 1's lifecycle/controller startup messages before dispatching. |
| Moving buggy never enters avoidance | Confirm the command used `/campus/master_command`, the master entered `NAVIGATING`, and safety reports an eligible obstacle. |
| Detour starts but stalls | Check action status, controller errors, `/odom`, and the local costmap; verify all three parameter values in step 4. |
| Status says `CLEAR` but buggy is still detouring | Wait for successful Nav2 completion and return to the route. |
| Unexpected obstacle immediately after startup | Allow the ground estimator to initialize before dispatching; see the warmup observation in the validation record. |

## Cleanup and repeatability

Remove the spawned models with:

```bash
ros2 run campus_nav test_obstacle_spawner --remove
```

For the initial comparisons, use one obstacle per fresh simulation. Each launch
overwrites `/tmp/camp_test.log`; copy it to another filename before restarting
if you need to compare runs. Stop the simulation using **Ctrl+C in Terminal 1**.

These tests place static objects into the scene while the buggy moves. They
do not test moving-agent prediction or arbitrary blocked-road scenarios.
