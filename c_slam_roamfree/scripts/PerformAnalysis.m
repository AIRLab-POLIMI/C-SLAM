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
pointsFHP = cell(numRectangles*4, 1);
for i = 1:numRectangles*4
    logFHP = tracksFHP{i}{2};
    if ~isempty(logFHP)
        [ti, qi] = getAnchorFrame(xFHP, logFHP);
        [rayi, omegai] = getHP(tracksFHP{i}{2});
        pointsFHP{i} =  hpcartesian(ti', qi, rayi', omegai)';      
    else
        pointsFHP{i} = [];
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


% compute FHP points error
deltaTracksFHP = zeros(numRectangles*4, 3);
deltaTracksFHPnorm = zeros(numRectangles*4, 1);
for i = 1:4:size(pointsFHP, 1)
    j = idivide(i, int32(4)) + 1;
    
    if ~isempty(pointsFHP{i})
        deltaTracksFHP(i, :) = landmarksGT(j, 1:3) - pointsFHP{i};
        deltaTracksFHP(i+1, :) = landmarksGT(j, 4:6) - pointsFHP{i+1};
        deltaTracksFHP(i+2, :) = landmarksGT(j, 7:9) - pointsFHP{i+2};
        deltaTracksFHP(i+3, :) = landmarksGT(j, 10:12) - pointsFHP{i+3};
        
        deltaTracksFHPnorm(i, :) = norm(deltaTracksFHP(i, :));
        deltaTracksFHPnorm(i+1, :) = norm(deltaTracksFHP(i+1, :));
        deltaTracksFHPnorm(i+2, :) = norm(deltaTracksFHP(i+2, :));
        deltaTracksFHPnorm(i+3, :) = norm(deltaTracksFHP(i+3, :));
    end
end


% compute rectangle points error
deltaTracksAR = zeros(numRectangles*4, 3);
deltaTracksARnorm = zeros(numRectangles*4, 1);
for i = 1:4:size(deltaTracksFHP, 1)
    
    j = idivide(i, int32(4)) + 1;
    rect = rectanglesAR{j};
    
    if ~isempty(rect)
        deltaTracksAR(i, :) = landmarksGT(j, 1:3) - rect(1, :);
        deltaTracksAR(i+1, :) = landmarksGT(j, 4:6) - rect(2, :);
        deltaTracksAR(i+2, :) = landmarksGT(j, 7:9) - rect(3, :);
        deltaTracksAR(i+3, :) = landmarksGT(j, 10:12) - rect(4, :);
        
        deltaTracksARnorm(i, :) = norm(deltaTracksAR(i, :));
        deltaTracksARnorm(i+1, :) = norm(deltaTracksAR(i+1, :));
        deltaTracksARnorm(i+2, :) = norm(deltaTracksAR(i+2, :));
        deltaTracksARnorm(i+3, :) = norm(deltaTracksAR(i+3, :));
    end
end

% compute width and ff errors
deltaDim = zeros(numRectangles, 2);
for i = 1:numRectangles
   if ~isempty(tracksAR{i}{2})
       logAR_Dim = tracksAR{i}{2};
       logAR_HP = tracksAR{i}{3};
       [wbari, ffi] = getDimensions(logAR_Dim);
       [~, omegai] = getHP(logAR_HP);

       wi = wbari/omegai;
       wgt = norm(landmarksGT(i, 1:3) - landmarksGT(i, 4:6));
       hgt = norm(landmarksGT(i, 1:3) - landmarksGT(i, 10:12));
       ffgt = wgt / hgt;

       deltaDim(i, :) = [wi - wgt, ffi - ffgt];
   end
end

%% Print trajectories and landmarks
figure(1)
clf
hold on
title('Map')

plotTrajectory(tFHP, qFHP, 'm');
plotTrajectory(tAR, qAR, 'b');
plotTrajectory(tgt, qgt, 'k');

for i = 1:size(landmarksGT,1)
   plotLandmark(landmarksGT(i, :), i, 'rx', 'k');   
end

%plot fhp 3d points
for i = 1:size(pointsFHP,1)
    pointToPlot = pointsFHP{i};
     if ~isempty(pointToPlot)
        plot3(pointToPlot(:, 1), pointToPlot(:, 2), pointToPlot(:, 3), 'mo');
     end
end

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
title('Translation norm error')

plot(deltaDeltaFHPnorm(:, 1),'m') 
plot(deltaDeltaARnorm(:, 1),'b')

% angles
figure(3)
clf
hold on
title('Angle norm error')

plot(deltaDeltaFHPnorm(:, 2),'m') 
plot(deltaDeltaARnorm(:, 2),'b')

% delta tracks
figure(4)
clf
hold on
title('Track norm error')

plot(deltaTracksFHPnorm, 'm');
plot(deltaTracksARnorm, 'b');

% delta w
figure(5)
clf
hold on
title('W error module');
plot(deltaDim(: , 1));

% delta dim
figure(6)
clf
hold on
title('ff error module'); 
plot(deltaDim(: , 2));

%% plot csv

% print deltaFHPnorm with timestamp

% discard the first pose
stampedDeltaDeltaFHPnorm = [xFHP(2:end-1, 1)-xFHP(2,1), deltaDeltaFHPnorm(2:end,:)];
csvwrite('./csv/deltaDeltaFHPnorm.csv', stampedDeltaDeltaFHPnorm)
stampedDeltaDeltaARnorm = [xAR(2:end-1, 1)-xFHP(2,1), deltaDeltaARnorm(2:end,:)];
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


% print tracks error norm

fN = 1-0.25*3/2:0.25:(numRectangles+0.25*3/2);

csvwrite('./csv/deltaTracksFHPnorm.csv', [fN' deltaTracksFHPnorm]);
csvwrite('./csv/deltaTracksARnorm.csv', [fN' deltaTracksARnorm]);

% print tracks error
csvwrite('./csv/deltaTracksFHP.csv', deltaTracksFHP);
csvwrite('./csv/deltaTracksAR.csv', deltaTracksAR);

% print dim error
csvwrite('./csv/deltaDim.csv', deltaDim);

