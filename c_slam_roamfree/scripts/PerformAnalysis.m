%% Script to analize SLAM performaces on FHP and AnchoredRectangle
clc
clf
clear

%% load pose results
addpath('./lib/math','./lib/parser', './lib/loaders', './lib/plot', './lib/time')
[x, xAR, xFHP] = loadDataset('.');
x(:, 1) = x(:, 1) / 1e9;

[tgt, qgt] = getPose(x, 'ros');
[tAR, qAR] = getPose(xAR, 'pose');
[tFHP, qFHP] = getPose(xFHP, 'pose');

% compute deltaDelta
deltaDeltaAR = cell(size(xAR, 1) - 1, 1);
timesAR = matchingTimes(xAR(:, 1)', x(:, 1)');
for i=2:size(timesAR, 2)
    T2 = poseMatrix(tAR(i, :)',  qAR(i, :));
    T1 = poseMatrix(tAR(i - 1, :)',  qAR(i - 1, :));
    deltaT = T1^-1*T2;
    
    j2 = timesAR(i);
    j1 = timesAR(i - 1);
    GT2 = poseMatrix(tgt(j2, :)',  qgt(j2, :));
    GT1 = poseMatrix(tgt(j1, :)',  qgt(j1, :));
    deltaGT = GT1^-1*GT2;
    
    deltaDeltaAR{i - 1} = deltaGT^-1 * deltaT;
end

deltaDeltaFHP = cell(size(xFHP, 1) - 1, 1);
timesFHP = matchingTimes(xFHP(:, 1)', x(:, 1)');
for i=2:size(timesFHP, 2)
    T2 = poseMatrix(tFHP(i, :)', qFHP(i, :));
    T1 = poseMatrix(tFHP(i - 1, :)', qFHP(i - 1, :));
    deltaT = T1^-1*T2;
    
    j2 = timesAR(i);
    j1 = timesAR(i - 1);
    GT2 = poseMatrix(tgt(j2, :)',  qgt(j2, :));
    GT1 = poseMatrix(tgt(j1, :)',  qgt(j1, :));
    deltaGT = GT1^-1*GT2;
    
    deltaDeltaFHP{i - 1} = deltaGT^-1 * deltaT;
end

%% Load landmarks results
numRectangles = 22;
landmarksGT = loadLandmarksGT('.');
tracksFHP = loadTracks('.', numRectangles*4); 
tracksAR = loadRectangles('.', numRectangles);

% load FHP points
pointsFHP = zeros(numRectangles*4, 3);
for i = 1:numRectangles*4
    logFHP = tracksFHP{i}{2};
    if ~isempty(logFHP)
        [ti, qi] = getAnchorFrame(xFHP, logFHP);
        [rayi, omegai] = getHP(tracksFHP{i}{2});
        pointsFHP(i, :) =  hpcartesian(ti', qi, rayi', omegai);      
    end
end

% load rectangles points
rectanglesAR = cell(numRectangles);
for i = 1:numRectangles
    logAR_Dim = tracksAR{i}{2};
    logAR_HP = tracksAR{i}{3};
    logAR_q = tracksAR{i}{4};
    if ~isempty(logAR_HP)
        [ti, qi] = getAnchorFrame(xAR, logAR_HP);
        [rayi, omegai] = getHP(logAR_HP);
        [wbari, ffi] = getDimensions(logAR_Dim);
        qri_f = getRectangleRotation(logAR_q);
        qri = quatprod(qi, qri_f);
        
        tri = hpcartesian(ti', qi, rayi', omegai);
        rectanglesAR{i} = ARcartesian(tri, qri, wbari, ffi, omegai);      
    else
        rectanglesAR{i} = [];
    end
end


%% Print trajectories and landmarks
figure(1)
clf
hold on

plotTrajectory(tFHP, qFHP, 'm');
plotTrajectory(tAR, qAR, 'b');
plotTrajectory(tgt, qgt, 'k');

for i = 1:size(landmarksGT,1)
   plotLandmark(landmarksGT(i, :), i, 'rx', 'k');   
end

%plot fhp 3d points
plot3(pointsFHP(:, 1), pointsFHP(:, 2), pointsFHP(:, 3), 'mo');

%plot rectangles
for i = 1:size(rectanglesAR,1)
    if ~isempty(rectanglesAR{i})
        rect = rectanglesAR{i};
        rect = [rect; rect(1, :)];
        line(rect(:,1), rect(:,2), rect(:,3), 'Color', 'r');
    end
end

axis equal
