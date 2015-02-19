function [ tracksAR ] = loadRectangles( path, numRectangles )
%LOADRECTANGLES Summary of this function goes here
%   Detailed explanation goes here

tracksAR = cell(numRectangles, 4);

for M = 0:numRectangles
    edgename = sprintf('Track_%d', M);
    
    edgef = sprintf('%s/dataset/AR/%s.log',path, edgename);
    Dimf = sprintf('%s/dataset/AR/%s%s.log',path, edgename,'_Dim');
    FOhpf = sprintf('%s/dataset/AR/%s%s.log',path, edgename,'_FOhp');
    FOqf = sprintf('%s/dataset/AR/%s%s.log',path, edgename,'_FOq');
    
    if exist(edgef, 'file')
        tracksAR{M+1}{1} = sortByT(stubbornLoad(edgef));
        tracksAR{M+1}{2} = sortByT(stubbornLoad(Dimf));
        tracksAR{M+1}{3} = sortByT(stubbornLoad(FOhpf));
        tracksAR{M+1}{4} = sortByT(stubbornLoad(FOqf));
    else
        tracksAR{M+1}{1} = [];
        tracksAR{M+1}{2} = [];
        tracksAR{M+1}{3} = [];
        tracksAR{M+1}{4} = [];
        fprintf('missing track %d: %s\n', M, edgef)
    end
end
end

