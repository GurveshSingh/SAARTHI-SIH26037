MAX_DETECTIONS = 64;
MAX_OBJECTS    = 64;
N_PRED_STEPS   = 15;
N_TRAJ_POINTS  = 50;

EgoState = makeBus({ ...
    'stamp',    'double', 1;
    'x',        'double', 1;
    'y',        'double', 1;
    'yaw',      'double', 1;
    'v',        'double', 1;
    'yaw_rate', 'double', 1});

Detection = makeBus({ ...
    'class_id',   'uint8',  1;
    'confidence', 'single', 1;
    'x',          'double', 1;
    'y',          'double', 1;
    'length',     'single', 1;
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
    'vx',       'double', 1;
    'vy',       'double', 1;
    'pos_cov',  'double', [2 2]});

ObjectList = makeBus({ ...
    'stamp', 'double',             1;
    'count', 'uint16',             1;
    'items', 'Bus: TrackedObject', MAX_OBJECTS});

Prediction = makeBus({ ...
    'track_id', 'uint32', 1;
    'class_id', 'uint8',  1;
    'x',        'double', N_PRED_STEPS;
    'y',        'double', N_PRED_STEPS;
    'radius',   'double', N_PRED_STEPS});

PredictionList = makeBus({ ...
    'stamp', 'double',          1;
    'dt',    'double',          1;
    'count', 'uint16',          1;
    'items', 'Bus: Prediction', MAX_OBJECTS});

BehaviorCmd = makeBus({ ...
    'state',        'uint8',  1;
    'target_speed', 'double', 1;
    'stop_s',       'double', 1});

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
    'linear_x',  'double', 1;
    'angular_z', 'double', 1});

clear MAX_DETECTIONS MAX_OBJECTS N_PRED_STEPS N_TRAJ_POINTS
fprintf("SAARTHI buses defined: EgoState, Detection(List), TrackedObject, ObjectList, " + ...
        "Prediction(List), BehaviorCmd, Trajectory, VehicleCommand\n");

function bus = makeBus(spec)
    els = Simulink.BusElement.empty;
    for k = 1:size(spec, 1)
        el = Simulink.BusElement;
        el.Name       = spec{k, 1};
        el.DataType   = spec{k, 2};
        el.Dimensions = spec{k, 3};
        els(end + 1) = el;
    end
    bus = Simulink.Bus;
    bus.Elements = els;
end
