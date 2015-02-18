function [x, xhatRectangles, xhatFHP] = loadDataset(path)
%LOADDATASET Load roamfree data of SLAM algorithm
%   Simply reads all the data from the dataset

poseGTFile = sprintf('%s/gt/%s',path, 'PoseSE3(W)_GT.log');
poseFileRectangles = sprintf('%s/dataset/AR/%s',path, 'PoseSE3(W).log');
poseFileFHP = sprintf('%s/dataset/FHP/%s',path, 'PoseSE3(W).log');


x = sortByT(stubbornLoad(poseGTFile));
xhatRectangles = sortByT(stubbornLoad(poseFileRectangles));
xhatFHP = sortByT(stubbornLoad(poseFileFHP));

end

