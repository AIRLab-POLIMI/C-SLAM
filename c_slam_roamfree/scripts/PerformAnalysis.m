%% Script to analize SLAM performaces on FHP and AnchoredRectangle
clc
clf
clear

%% load pose results
addpath('./lib/math','./lib/parser', './lib/loaders', './lib/plot', './lib/time')
[x, xAR, xFHP] = loadDataset('.');
x(:, 1) = x(:, 1) / 1e9;

[tgt, qgt_wr] = getPose(x, 'ros');
[tAR, qAR] = getPose(xAR, 'pose');
[tFHP, qFHP] = getPose(xFHP, 'pose');
q_rc = [0.5, -0.5, 0.5, -0.5];

%rotate the gt to be oriented like the camera camera
qgt = zeros(size(qgt_wr,1), 4);
for i = 1:size(qgt_wr,1)
   qgt(i, :) = quatprod(qgt_wr(i, :), q_rc);
end

% compute deltaDelta
deltaDeltaAR = cell(size(xAR, 1) - 1, 1);
timesAR = matchingTimes(xAR(:, 1)', x(:, 1)');
deltaDeltaARnorm = zeros(size(xAR, 1) - 1, 1);
deltaARnorm = zeros(size(xAR, 1) - 1, 1); 
deltaGTnorm = zeros(size(xAR, 1) - 1, 1);
for i=2:size(timesAR, 2)
    T2 = poseMatrix(tAR(i, :)',  qAR(i, :));
    T1 = poseMatrix(tAR(i - 1, :)',  qAR(i - 1, :));
    deltaT = T1^-1*T2;
    
    deltaARnorm(i - 1) = norm(tAR(i, :) - tAR(i - 1, :));
    
    j2 = timesAR(i);
    j1 = timesAR(i - 1);
    GT2 = poseMatrix(tgt(j2, :)',  qgt(j2, :));
    GT1 = poseMatrix(tgt(j1, :)',  qgt(j1, :));
    deltaGT = GT1^-1*GT2;
    
    deltaGTnorm(i - 1) = norm(tgt(j2, :) - tgt(j1, :));
    
    deltaDeltaAR{i - 1} = deltaGT^-1 * deltaT;
    deltaDeltaARnorm(i - 1) = norm(deltaDeltaAR{i-1}(1:3,4));
end

deltaDeltaFHP = cell(size(xFHP, 1) - 1, 1);
timesFHP = matchingTimes(xFHP(:, 1)', x(:, 1)');
deltaDeltaFHPnorm = zeros(size(xFHP, 1) - 1, 1);
deltaFHPnorm = zeros(size(xAR, 1) - 1, 1); 

for i=2:size(timesFHP, 2)
    T2 = poseMatrix(tFHP(i, :)', qFHP(i, :));
    T1 = poseMatrix(tFHP(i - 1, :)', qFHP(i - 1, :));
    deltaT = T1^-1*T2;
    
    deltaFHPnorm(i - 1) = norm(tFHP(i, :) - tFHP(i - 1, :));
    
    j2 = timesAR(i);
    j1 = timesAR(i - 1);
    GT2 = poseMatrix(tgt(j2, :)',  qgt(j2, :));
    GT1 = poseMatrix(tgt(j1, :)',  qgt(j1, :));
    deltaGT = GT1^-1*GT2;
    
    deltaDeltaFHP{i - 1} = deltaGT^-1 * deltaT;
    deltaDeltaFHPnorm(i - 1) = norm(deltaDeltaFHP{i-1}(1:3,4));
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
        line(rect(:,1), rect(:,2), rect(:,3), 'Color', 'b');
    end
end

axis equal

%% plot delta comparison

figure(2)
hold on

plot(deltaDeltaFHPnorm,'m') 
plot(deltaDeltaARnorm,'b')


figure(3)
hold on

plot(deltaARnorm, 'b');
plot(deltaGTnorm, 'k');
plot(deltaFHPnorm, 'm');
