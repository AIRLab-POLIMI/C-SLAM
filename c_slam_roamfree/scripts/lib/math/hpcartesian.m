function [ cc ] = hpcartesian( t, q, ray, omega )
%HPCARTESIAN takes in input an homogeneus point and return cartesian
%   Using an homogeneus point parametrization, retrive it's cartesian
%   coordinates.

cc = t + 1/omega*quatrot(q)*ray;

end

