% Load reference path for recorded drive segment
xData   = load('refPosesX.mat');
yData   = load('refPosesY.mat');
yawData = load('refPosesT.mat');

% Set up workspace variables used by model
refPosesX = xData.refPosesX;
refPosesY = yData.refPosesY;
refPosesT = yawData.refPosesT;

% Display path on scene image
sceneName = 'USHighway';
hScene = figure;
helperShowSceneImage(sceneName);
hold on
scatter(refPosesX(:,2), refPosesY(:,2), 7, 'filled')

% Adjust axes limits
xlim([-150 100])
ylim([-125 75])