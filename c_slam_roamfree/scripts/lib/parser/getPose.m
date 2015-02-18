function [ t, q ] = getPose(log, type)
%GETPOSE get Pose from log file
%   Detailed explanation goes here

switch type
    case 'pose'
        t = log(:, 3:5);
        q = log(:, 6:9);
        
    case 'ros'
        t = log(:, 4:6);
        q = log(:, 7:10);
        
    otherwise
        t = [];
        q = [];
        
end


end

