function [ landmarksGT ] = loadLandmarksGT( path )
%LOADLANDMARKSGT function used to load landmarks GT
%   Detailed explanation goes here

landmarksGTFile = sprintf('%s/gt/%s',path, 'LandmarksGT.log');
landmarksGT = stubbornLoad(landmarksGTFile);

end

