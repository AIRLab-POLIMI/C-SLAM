%% Script to analize SLAM performaces on FHP and AnchoredRectangle

% load pose results
addpath('./lib/math','./lib/parser')
[x, xAR, xFHP] = loadDataset('.');
x(:, 1) = x(:, 1) / 1e9;

[tgt, qgt] = getPose(x, 'ros');
[tAR, qAR] = getPose(xAR, 'pose');
[tFHP, qFHP] = getPose(xFHP, 'pose');

deltaDeltaAR = cell(size(xAR, 1) - 1, 1);
timesAR = matchingTimes(xhatRectangles(:, 1)', x(:, 1)');
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

