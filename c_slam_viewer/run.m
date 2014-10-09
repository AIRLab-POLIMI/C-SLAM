addpath('lib')

global path
global stopAll
    
stopAll = 1;

pause(1)

stopAll = 0;

path = '/tmp/roamfree/';

watchFile();

close all
figure(1)
figure(2)   
