function r = watchFile()

period = 0.5; %seconds between directory checks

cb = @(x,y)(hasChanged(x, @plotStuff));
%cb = @(x,y)(hasChanged(x, @plotTrajectory));

t = timer('TimerFcn', cb, 'Period', period, 'executionmode', 'fixedrate');
start(t);

r = 0;

end

function s = hasChanged(t, callback)

persistent lasttime

global path

[s,r] = unix(sprintf('stat -c %%y "%s%s"',path, 'PoseSE3(W).log'));
%[s,r] = unix(sprintf('stat -c %%y "%s%s"',path, 'alpha.log'));
if s == 0    
    curtime = str2double(r(18:30));

    if curtime ~= lasttime
        display('Files have changed')
        callback();
    end

    lasttime = curtime;
end

global stopAll

if stopAll == 1
    display('Exiting')
    stop(t)
end
    
end


