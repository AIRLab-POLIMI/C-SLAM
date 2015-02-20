function [ t, q ] = getPose(log, type)
%GETPOSE get Pose from log file
%   Detailed explanation goes here

switch type
    case 'pose'
        t = log(:, 3:5);
        q = log(:, 6:9);
        
    case 'ros'
        t = log(:, 4:6);
        q = [log(:,10) log(:, 7:9)];
        
    otherwise
        t = [];
        q = [];
        
end


end

