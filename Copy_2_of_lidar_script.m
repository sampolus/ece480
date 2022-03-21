% Load variables to workspace if they do not exist
if exist('refPoses', 'var')==0 || exist('wayPoints', 'var')==0

    % Load MAT-file containing preselected waypoints
    data = load('waypointsCurvedroad');
    data = data.waypointsCurvedroad;

    % Assign to caller workspace
    assignin('caller', 'wayPoints', {data.wayPoints});
    assignin('caller', 'refPoses', {data.refPoses});
end

numPoses = size(refPoses{1}, 1);

refDirections  = ones(numPoses,1);   % Forward-only motion
numSmoothPoses = 20 * numPoses;      % Increase this to increase the number of returned poses

[smoothRefPoses,~,cumLengths] = smoothPathSpline(refPoses{1}, refDirections, numSmoothPoses);

% % Configure the model to stop simulation at 5 seconds.
simStopTime = 50;
set_param(gcs, 'StopTime', num2str(simStopTime));

% Create a constant velocity profile by generating a time vector
% proportional to the cumulative path length.
timeVector = normalize(cumLengths, 'range', [0, simStopTime]);

% Create variables required by the Simulink model.
refPosesX = [timeVector, smoothRefPoses(:,1)];
refPosesY = [timeVector, smoothRefPoses(:,2)];
refPosesT = [timeVector, smoothRefPoses(:,3)];

% Display path on scene image
sceneName = 'Curvedroad';
[sceneImage, sceneRef] = helperGetSceneImage(sceneName);
hScene = figure;
hold on
scatter(refPosesX(:,2), refPosesY(:,2), 7, 'filled')

% Adjust axes limits
xlim([-150 100])
ylim([-125 75])

close(hScene)

if ~ispc
    error(['3D Simulation is only supported on Microsoft', char(174), ' Windows', char(174), '.']);
end

% Open the model
modelName = 'Copy_2_of_LidarModel';
open_system(modelName);
snapnow;

%open_system([modelName, '/Simulation 3D Lidar'], 'force');

% Update simulation stop time to end when reference path is completed
simStopTime = refPosesX(end,1);
set_param(gcs, 'StopTime', num2str(simStopTime));

% Load INS data from MAT file
data = load('insMeasurement.mat');
insData = data.insMeasurement.signals.values;

% Run the simulation
simOut = sim(modelName);

% Create a pointCloud array from the recorded data
ptCloudArr = helperGetPointCloud(simOut);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set the random seed for example reproducibility
rng(0);

% Create a lidar map builder
mapBuilder = helperLidarMapBuilder('DownsamplePercent', 0.25, ...
    'RegistrationGridStep', 3.5, 'Verbose', true);

% Configure the map builder to detect loop closures
configureLoopDetector(mapBuilder, ...
    'LoopConfirmationRMSE',  2, ...
    'SearchRadius',          0.15, ...
    'DistanceThreshold',     0.15);

% Loop through the point cloud array and progressively build a map
skipFrames = 5;
numFrames  = numel(ptCloudArr);
exitLoop   = false;

prevInsMeas = insData(1, :);
for n = 1 : skipFrames : numFrames

    insMeas = insData(n, :);

    % Estimate initial transformation using INS
    initTform = helperEstimateRelativeTransformationFromINS(insMeas, prevInsMeas);

    % Update map with new lidar frame
    updateMap(mapBuilder, ptCloudArr(n), initTform);

    % Update top-view display
    isDisplayOpen = updateDisplay(mapBuilder, exitLoop);

    % Check and exit if needed
    exitLoop = ~isDisplayOpen;

    prevInsMeas = insMeas;
end

snapnow;

% Close display
closeDisplay = true;
updateDisplay(mapBuilder, closeDisplay);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualize viewset before pose graph optimization
hFigViewset = figure;
hG = plot(mapBuilder.ViewSet);
view(hG.Parent, 2);
title('Viewset Display')

% Optimize pose graph and rebuild map
optimizeMapPoses(mapBuilder);
rebuildMap(mapBuilder);

% Overlay viewset after pose graph optimization
hold(hG.Parent, 'on');
plot(mapBuilder.ViewSet);
hold(hG.Parent, 'off');

legend(hG.Parent, 'before', 'after')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% close(hFigViewset)

hFigMap = figure;
pcshow(mapBuilder.Map)

% Customize axes labels and title
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Point Cloud Map')

helperMakeFigurePublishFriendly(hFigMap);

% Close windows
% close(hFigMap)
% close_system(modelName)
