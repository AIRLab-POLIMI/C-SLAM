function [ euler ] = extractEuler( T )
%EXTRACTEULER Summary of this function goes here
%   Detailed explanation goes here
q = qGetQ(T(1:3, 1:3));
euler = quatToEuler(q');

end

