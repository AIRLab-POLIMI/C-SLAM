addpath('lib')

global path
global stopAll
    
stopAll = 1;

pause(1)

stopAll = 0;

path = '/tmp/roamfree/';

close all
figure(1)
figure(2)
set(2, 'DefaultAxesPosition', [0.05, 0.05, 0.9, 0.9])

watchFile();
