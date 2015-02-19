function [ t, q ] = getAnchorFrame( logPose, logEdge )
%GETANCHORFRAME Return an anchor frame of
%   Retrun anchor frame of the specified edge type

ai = find(logPose(:,1) == logEdge(1),1);
AF = logPose(ai, :);
t = AF(3:5);
q = AF(6:9);

end

