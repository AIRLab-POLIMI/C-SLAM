function [ ray, omega ] = getHP( logHP )
%GETRAYANDDEPTH Returns homogeneus point and inverse scale
%   given a log, returns the homogeneus point and it's inverse scale
ray = [logHP(3), logHP(4), 1];
omega = logHP(5);

end

