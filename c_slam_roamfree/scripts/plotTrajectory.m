function plotTrajectory( t, q, color )
%PLOTTRAJECTORY plot estimated trajectory 
%   Given a translation vector t and a quaternion vector q plot estimated
%   trajectory

%i = find(x(:,2) == max(x(:,2)));

plot3(t(:, 1), t(:, 2), t(:, 3), color);
%plot3(x(1:i(1),3),x(1:i(1),4),x(1:i(1),5),'k');

%for j = i(1:20:length(i))'    
%for j = [size(t,1):-1:1 1]
%    plotAxis(t(j)',q(6:9),1)    
%end


end

