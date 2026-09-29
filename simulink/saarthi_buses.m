%SAARTHI_BUSES  Define the frozen Simulink bus contracts for the SAARTHI autonomy model.
%   Run this script before opening saarthi_autonomy.slx. It creates every bus
%   object in the base workspace. Fixed-size arrays with a count field keep the
%   model code-generation friendly (Simulink Coder / ROS 2 node).

MAX_DETECTIONS = 64;    % detections per perception frame
MAX_OBJECTS    = 64;    % confirmed tracks
N_PRED_STEPS   = 15;    % prediction horizon: 15 x 0.2 s = 3 s
N_TRAJ_POINTS  = 50;    % points in a planned trajectory

% Class ids shared with the perception nodes (YOLO, 9 Indian classes)
%   1 car  2 bus  3 truck  4 auto-rickshaw  5 two-wheeler
%   6 bicycle  7 pedestrian  8 pushcart  9 cattle

EgoState = makeBus({ ...
    'stamp',    'double', 1;   % sim time [s]
    'x',        'double', 1;   % map frame [m]
    'y',        'double', 1;
    'yaw',      'double', 1;   % [rad]
    'v',        'double', 1;   % [m/s]
    'yaw_rate', 'double', 1}); % [rad/s]

Detection = makeBus({ ...
    'class_id',   'uint8',  1;
    'confidence', 'single', 1;
    'x',          'double', 1;   % map frame [m]
    'y',          'double', 1;
    'length',     'single', 1;   % [m]
    'width',      'single', 1});

DetectionList = makeBus({ ...
    'stamp', 'double',         1;
    'count', 'uint16',         1;
    'items', 'Bus: Detection', MAX_DETECTIONS});

TrackedObject = makeBus({ ...
    'track_id', 'uint32', 1;
    'class_id', 'uint8',  1;
    'x',        'double', 1;
    'y',        'double', 1;
    'vx',       'double', 1;   % [m/s]
    'vy',       'double', 1;
    'pos_cov',  'double', [2 2]});

ObjectList = makeBus({ ...
    'stamp', 'double',             1;
    'count', 'uint16',             1;
    'items', 'Bus: TrackedObject', MAX_OBJECTS});

Prediction = makeBus({ ...
    'track_id', 'uint32', 1;
    'class_id', 'uint8',  1;
    'x',        'double', N_PRED_STEPS;   % predicted centre per step
    'y',        'double', N_PRED_STEPS;
    'radius',   'double', N_PRED_STEPS}); % capsule radius per step (grows per class)

PredictionList = makeBus({ ...
    'stamp', 'double',          1;
    'dt',    'double',          1;        % 0.2 s
    'count', 'uint16',          1;
    'items', 'Bus: Prediction', MAX_OBJECTS});

BehaviorCmd = makeBus({ ...
    'state',        'uint8',  1;   % 1 Cruise 2 Follow 3 Pass 4 Yield 5 Creep 6 Go 7 Merge 8 EmergencyStop
    'target_speed', 'double', 1;   % [m/s]
    'stop_s',       'double', 1}); % stop point along the reference path [m], NaN if none

Trajectory = makeBus({ ...
    'stamp',     'double', 1;
    'valid',     'boolean', 1;
    'x',         'double', N_TRAJ_POINTS;
    'y',         'double', N_TRAJ_POINTS;
    'yaw',       'double', N_TRAJ_POINTS;
    'v',         'double', N_TRAJ_POINTS;
    'curvature', 'double', N_TRAJ_POINTS;
    't',         'double', N_TRAJ_POINTS});

VehicleCommand = makeBus({ ...
    'linear_x',  'double', 1;   % [m/s]  -> /cmd_vel linear.x
    'angular_z', 'double', 1}); % [rad/s] -> /cmd_vel angular.z

clear MAX_DETECTIONS MAX_OBJECTS N_PRED_STEPS N_TRAJ_POINTS
fprintf("SAARTHI buses defined: EgoState, Detection(List), TrackedObject, ObjectList, " + ...
        "Prediction(List), BehaviorCmd, Trajectory, VehicleCommand\n");

function bus = makeBus(spec)
%MAKEBUS  Build a Simulink.Bus from {name, dataType, dimensions} rows.
    els = Simulink.BusElement.empty;
    for k = 1:size(spec, 1)
        el = Simulink.BusElement;
        el.Name       = spec{k, 1};
        el.DataType   = spec{k, 2};
        el.Dimensions = spec{k, 3};
        els(end + 1) = el; %#ok<AGROW>
    end
    bus = Simulink.Bus;
    bus.Elements = els;
end
