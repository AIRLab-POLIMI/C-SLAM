function r = plotTrajectory()

global path

global x Sa edge augs 

%% load stuff

posef = sprintf('%s%s',path, 'PoseSE3(W).log');
x = sortByT(stubbornLoad(posef));

clf

%plotGT

hold on

%% plot trajectory

i = find(x(:,2) == max(x(:,2)));

plot3(x(i,3),x(i,4),x(i,5),'m');
plot3(x(1:i(1),3),x(1:i(1),4),x(1:i(1),5),'k');

%for j = i(1:5:length(i))'    
for j = length(x):-20:1
    plotAxis(x(j,3:5)',x(j,6:9),0.1)    
end    

%% plot the GPS

GPS = load('/home/davide/Code/ros/src/quadrivio/bags/The_Bag_GPS.dat');

ii = find(GPS(:,1) < x(end-1,1));
plot3(GPS(ii,2),GPS(ii,3),GPS(ii,4),'rx')

grid on

axis image