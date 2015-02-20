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

% rotate the gt to be oriented like the camera camera
qgt = zeros(size(qgt_wr,1), 4);
for i = 1:size(qgt_wr,1)
   qgt(i, :) = quatprod(qgt_wr(i, :), q_rc);
end

% compute deltaDelta
deltaDeltaAR = cell(size(xAR, 1) - 1, 1);
timesAR = matchingTimes(xAR(:, 1)', x(:, 1)');
deltaDeltaARnorm = zeros(size(xAR, 1) - 1, 2);

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
    deltaDeltaARnorm(i - 1, 1) = norm(extractTranslation(deltaDeltaAR{i-1}));
    deltaDeltaARnorm(i - 1, 2) = norm(extractEuler(deltaDeltaAR{i-1}));
end

deltaDeltaFHP = cell(size(xFHP, 1) - 1, 1);
timesFHP = matchingTimes(xFHP(:, 1)', x(:, 1)');
deltaDeltaFHPnorm = zeros(size(xFHP, 1) - 1, 2);

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
    deltaDeltaFHPnorm(i - 1, 1) = norm(extractTranslation(deltaDeltaFHP{i-1}));
    deltaDeltaFHPnorm(i - 1, 2) = norm(extractEuler(deltaDeltaFHP{i-1}));
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

% translation
figure(2)
clf
hold on

plot(deltaDeltaFHPnorm(:, 1),'m') 
plot(deltaDeltaARnorm(:, 1),'b')

% angles
figure(3)
clf
hold on

plot(deltaDeltaFHPnorm(:, 2),'m') 
plot(deltaDeltaARnorm(:, 2),'b')


%% plot csv

% print deltaFHPnorm with timestamp

stampedDeltaDeltaFHPnorm = [xFHP(1:end-1, 1), deltaDeltaFHPnorm];
csvwrite('./csv/deltaDeltaFHPnorm.csv', stampedDeltaDeltaFHPnorm)
stampedDeltaDeltaARnorm = [xAR(1:end-1, 1), deltaDeltaARnorm];
csvwrite('./csv/deltaDeltaARnorm.csv', stampedDeltaDeltaARnorm)

% print euler and transation with timestamp

stampedDeltaDeltaFHP = zeros(size(deltaDeltaFHP, 1), 7);
for i = 1:size(deltaDeltaFHP, 1)
    T = deltaDeltaFHP{i};
    t = extractTranslation(T);
    euler = extractEuler(T);
    stampedDeltaDeltaFHP(i, :) = [ xFHP(i, 1), t', euler ];
    
end

csvwrite('./csv/deltaDeltaFHP.csv', stampedDeltaDeltaFHP)

stampedDeltaDeltaAR = zeros(size(deltaDeltaAR, 1), 7);
for i = 1:size(deltaDeltaAR, 1)
    T = deltaDeltaAR{i};
    t = extractTranslation(T);
    euler = extractEuler(T);
    stampedDeltaDeltaAR(i, :) = [ xAR(i, 1), t', euler ];
end

csvwrite('./csv/deltaDeltaAR.csv', stampedDeltaDeltaAR)


