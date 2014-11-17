addpath('lib');

res = get(0,'screensize');

global path
global stopAll
    
stopAll = 1;

pause(1);

stopAll = 0;

path = '/tmp/roamfree/';



close all
figure(1)
set(1,'OuterPosition', [0 0 res(3)/2 res(4)])

figure(2)  
set(2,'OuterPosition', [res(3)/2 0 res(3)/2 res(4)])


watchFile();
