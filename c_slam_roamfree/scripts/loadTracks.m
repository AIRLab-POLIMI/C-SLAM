function [ tracksFHP ] = loadTracks( path, numtracks )
%LOADTRACKS Summary of this function goes here
%   Detailed explanation goes here
for M = 0:numtracks
    tracksFHP = cell(numtracks, 2);
    edgename = sprintf('Track_%d', M);
    
    edgef = sprintf('%s/dataset/FHP/%s.log',path, edgename);
    HPf = sprintf('%s/dataset/FHP/%s%s.log',path, edgename,'_HP');
    
    if exist(edgef, 'file')
        tracksFHP{M+1}{1} = sortByT(stubbornLoad(edgef));
        tracksFHP{M+1}{2} = sortByT(stubbornLoad(HPf));
    else
        fprintf('missing track %d: %s\n', M, edgef)
    end
end

end

