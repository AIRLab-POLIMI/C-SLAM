function [ T ] = poseMatrix(t, q)
%POSEMATRIX take translation and quaternion, outputs transformation matrix
%   Detailed explanation goes here
R = quatrot(q);
T = [ R , t; 0, 0, 0, 1];
end

