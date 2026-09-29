# SAARTHI Simulink autonomy

This folder holds the MATLAB/Simulink side of SAARTHI: tracking, prediction, decision, planning and control. The ROS 2 workspace in `../ros2_ws` provides the world, the sensors, perception and the vehicle interface. This side does the thinking.

> **Status:** POCs 1–6 run in simulation. The interface below is frozen. Model files (`.slx`) are committed here as each layer is finalised; until then this folder holds the interface spec and the bus definitions every layer builds against.

## Requirements

MATLAB R2024a or newer, with:

| Toolbox | Used for |
|---|---|
| Simulink, Stateflow | the model and the behaviour chart |
| ROS Toolbox | ROS 2 subscribe / publish blocks, sim time |
| Sensor Fusion and Tracking Toolbox | `trackerGNN`, `trackerJPDA`, `trackingEKF` |
| Navigation Toolbox | `trajectoryOptimalFrenet`, `plannerHybridAStar`, `dynamicCapsuleList`, `binaryOccupancyMap` |
| Automated Driving Toolbox | Stanley lateral / longitudinal controllers |
| Simulink Test, Simulink Coder (optional) | randomised test runs; C++ / ROS 2 node generation |

## Model structure

```
saarthi_autonomy.slx
├── ROS 2 In          /clock, /odom, /detections, /drivable_grid      → EgoState, DetectionList, grid
├── Track + Predict   trackerGNN / trackerJPDA, CV + CTRV models      → ObjectList, PredictionList (15 × 0.2 s)
├── Capsule World     dynamicCapsuleList, radius grows per class      → collision checker
├── Behaviour         Stateflow: Cruise ⇄ Follow ⇄ Pass,
│                     Yield → Creep → Go, Merge, EmergencyStop (from any state) → BehaviorCmd
├── Planner           trajectoryOptimalFrenet (full road width),
│                     plannerHybridAStar fallback                     → Trajectory (50 points)
├── Controller        Stanley + PID on a kinematic bicycle model      → VehicleCommand
└── ROS 2 Out         /cmd_vel (50 Hz), /planned_path, /behaviour_state
```

## Interface (frozen)

**ROS 2 topics**

| Direction | Topic | Type | Rate |
|---|---|---|---|
| in | `/clock` | `rosgraph_msgs/Clock` | every sim step |
| in | `/odom` | `nav_msgs/Odometry` | 50 Hz |
| in | `/detections` | `vision_msgs/Detection3DArray` | 10 Hz |
| in | `/drivable_grid` | `nav_msgs/OccupancyGrid` | 10 Hz |
| out | `/cmd_vel` | `geometry_msgs/Twist` | 50 Hz |
| out | `/planned_path` | `nav_msgs/Path` | 10 Hz |
| out | `/behaviour_state` | `std_msgs/String` | 10 Hz |

**Timing:** Gazebo owns time. Every ROS 2 node runs with `use_sim_time:=true`, and Simulink reads `/clock` with a 10 ms fixed step. Planning runs at 10 Hz and control at 50 Hz, with rate-transition blocks between them.

**Buses** are defined in [`saarthi_buses.m`](saarthi_buses.m). Run it before opening any model:

```matlab
run("saarthi_buses.m")   % creates EgoState, Detection, DetectionList, TrackedObject,
                         % ObjectList, Prediction, PredictionList, BehaviorCmd,
                         % Trajectory and VehicleCommand in the base workspace
```

**Enumerations used in the buses**

| `class_id` | Class | | `BehaviorCmd.state` | State |
|---|---|---|---|---|
| 1 | car | | 1 | Cruise |
| 2 | bus | | 2 | Follow |
| 3 | truck | | 3 | Pass |
| 4 | auto-rickshaw | | 4 | Yield |
| 5 | two-wheeler | | 5 | Creep |
| 6 | bicycle | | 6 | Go |
| 7 | pedestrian | | 7 | Merge |
| 8 | pushcart | | 8 | EmergencyStop |
| 9 | cattle | | | |

Units: positions in metres (map frame), angles in radians, speeds in m/s, time in seconds (sim time). Fixed-size arrays with a `count` field keep the model ready for code generation. Limits: 64 detections, 64 tracks, 15 prediction steps of 0.2 s (`dt`), and 50 trajectory points. `stop_s` is NaN when there is no stop point.

## Class-aware prediction

Every tracked agent is rolled forward 15 steps of 0.2 s. Each step's capsule radius grows at a per-class rate, so the planner keeps the most distance from the least predictable road users:

| Class | Growth (relative) |
|---|---|
| Cattle | highest |
| Pedestrian | high |
| Two-wheeler, auto-rickshaw | medium |
| Car, bus, truck | low |
| Parked / static | none |

## Scenarios

Scenarios currently run in Gazebo (`../ros2_ws/src/simulation`). RoadRunner scenes (village road, unsignalised junction) plug in behind the same ROS 2 interface, so the models don't change.
