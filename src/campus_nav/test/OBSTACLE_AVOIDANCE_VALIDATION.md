# Obstacle avoidance regression — 2026-10-16

The user confirmed on 2026-10-16 that the supplied full-simulation test
procedure ran correctly. This confirmation supplements the development
trials below; it does not assert that every campus route or obstacle layout
has been tested. The complete operator procedure is in [TESTING.md](../TESTING.md).

## Reproduced behavior

Using `GOAL:Chemistry Block` on `/campus/master_command` from the default
LOC spawn, then spawning a pedestrian 6m ahead of the moving buggy:

- The original master **did** enter `OBSTACLE_AVOIDANCE` and disable Pure Pursuit.
- Nav2 drove almost straight toward the obstacle, stopped, and eventually
  reported `Failed to make progress` (action status 6).
- Before spawning anything, roughly 7,000–10,000 of the 22,500 local costmap
  cells were occupied. The road itself was being marked as an obstacle.

## Focused correction

Only two runtime files in `campus_nav` changed:

1. `config/nav2_params.yaml`: raise the obstacle height threshold from 0.10m
   to 0.45m, increase inflation from 0.8m to 1.2m, and enable footprint collision
   checking in MPPI. The threshold is specific to this campus world's road at
   map Z=0.30m: it excludes road and speed-breaker returns while retaining the
   dog and pedestrian models. Nav2 applies height filtering **after** transforming
   the cloud to the global frame; see the
   [Jazzy observation buffer implementation](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/observation_buffer.cpp).
2. `campus_nav/odom_bridge_node.py`: preserve the measured angular Z velocity
   sign. A fixed yaw rotation between the CAD and navigation frames does not
   reverse the Z axis. The previous sign made velocity feedback disagree with
   the derivative of the published vehicle heading.

The costmap changes alone did not solve the reproduced stall. Adding the
yaw-rate correction allowed the existing detour and handoff to complete.
The planner, Pure Pursuit, safety classifier, mission FSM, and Saye reference
packages were not edited.

## Verification

- `colcon build --packages-select campus_nav --symlink-install`: passed.
- `test_odom_bridge.py`: 9 passed, covering positive/negative yaw rates,
  stationary heading, and angle wraparound. Against the original bridge,
  the six turning cases fail and the three zero-rate cases pass.
- All 30 Python files in `campus_nav` parsed successfully.
- Headless Gazebo trials used the existing world, vehicle, sensor bridges,
  planner, lane detector, safety monitor, follower, master, odometry bridge,
  and Nav2 controller. RViz, window arrangement, FAST-LIO, and audit logging
  were omitted from the temporary test launch. Separate ROS domains and
  Gazebo partitions isolated the trials.
- Pedestrian trial with the corrected files: one avoidance entry, successful
  Nav2 completion, return to Pure Pursuit, and continued driving along the
  original route. No progress failure. Repeated successfully in a fresh world.
- Dog trial: the low obstacle remained visible in the costmap; one avoidance
  entry, successful Nav2 completion, and return to Pure Pursuit. No progress
  failure.
- Speed-breaker trial after sensor initialization: `SLOWDOWN_BUMP` was reported,
  Pure Pursuit remained enabled, the buggy passed the bump at its existing
  0.8m/s command, and no avoidance action was triggered.
- Before obstacles appeared, the corrected local costmap had zero occupied
  cells on the tested clear road. Normal driving commanded 0.8m/s both before
  avoidance and after rejoining, matching the existing follower behavior.

These are simulated spawned obstacles, not moving-agent prediction tests or
hardware validation. No contact sensor was added; passing was assessed using
vehicle pose, obstacle position, action completion, and continued travel.
An initial speed-breaker trial dispatched during ground-estimator warmup and
triggered a false obstacle before the test object existed. It was excluded
from the bump comparison and repeated after 20 seconds of initialization.
That separate startup behavior is outside this focused correction.

## Repeat

For build, launch, readiness checks, pedestrian/dog/speed-breaker cases,
telemetry, and cleanup, use [the full test guide](../TESTING.md).

Source ROS and the built workspace, launch the usual full simulation, and wait
for sensor initialization and Nav2 activation before dispatching:

```bash
ros2 topic pub --once /campus/master_command std_msgs/msg/String \
  "{data: 'GOAL:Chemistry Block'}"
```

Once the buggy is moving on a clear stretch of road, test one obstacle per
fresh run:

```bash
ros2 run campus_nav test_obstacle_spawner --type human --distance 6.0
# Or:
ros2 run campus_nav test_obstacle_spawner --type dog --distance 6.0
```

Expect `NAVIGATING → OBSTACLE_AVOIDANCE → NAVIGATING`, a lateral pass, and
`Nav2 avoidance complete` in the master log. Merely entering avoidance or
seeing `CLEAR` after the obstacle leaves the forward corridor is not sufficient
evidence of a successful pass.

Run the isolated velocity regression without Gazebo:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest src/campus_nav/test/test_odom_bridge.py -q
```
