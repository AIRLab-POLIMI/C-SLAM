function r = plotStuff()

global path

global x Sa edge augs 

%% load stuff

posef = sprintf('%s%s',path, 'PoseSE3(W).log');
x = sortByT(stubbornLoad(posef) );

clf

subplot(2,5,[1 6])
hold on

%% plot trajectory
    
i = find(x(:,2) == max(x(:,2)));

plot3(x(i,3),x(i,4),x(i,5),'m');
plot3(x(1:i(1),3),x(1:i(1),4),x(1:i(1),5),'k');

%for j = i(1:5:length(i))'    
for j = size(x,1):-20:1        
    plotAxis(x(j,3:5)',x(j,6:9),0.1)    
end

%% plot markers
hold on

for i = 0:15
    edgef = sprintf('%sMarker_%d_Fposition.log',path, i);
    edgef2 = sprintf('%sMarker_%d_Forientation.log',path, i);
    
    if exist(edgef, 'file')
        Mx = sortByT(stubbornLoad(edgef));        
        
        if exist(edgef2, 'file')
            Mq = sortByT(stubbornLoad(edgef2));                        
            plotAxis(Mx(1,3:5)',Mq(1,3:6),0.25);
        else 
            plot3(Mx(1,3),Mx(1,4),Mx(1,5), 'rx');
        end
        text(Mx(1,3),Mx(1,4),Mx(1,5), sprintf('M%d',i));
    end
    
end

%% plot covariances


for j = 1:size(x,1)    
   if (~isinf(x(j,10)))
       plot(x(j,3),x(j,4),'x')
       
%       plotErrorEllipse(x(j,3),x(j,4),[ x(j,[10 11]); x(j,[11 16]) ], 'r');
   end
end

axis image

%% -------------------- SENSORI

edgename = 'IMUintegral';

edgef = sprintf('%s%s.log',path, edgename);

if exist(edgef, 'file')
    
    edge = sortByT(stubbornLoad(edgef));
    oi = find(edge(:,1) >= x(i(1),1),1);
    
    axis tight    
    %% plot error
    subplot(2,5,2)
    
    %plot(edge(oi:end,1),edge(oi:end,25:27))    
    plot(edge(:,1) - edge(1,1),edge(:,76:78))    

    axis tight

    %% plot measure
    subplot(2,5,7)

    %plot(edge(oi:end,1),[edge(oi:end,22:24)])
    plot(edge(:,1) - edge(1,1),[edge(:,23:25)])
    
    axis tight
end

%% -------------------- LANDMARK

for M = 0:2

    edgename = sprintf('Track_%d', M);

    edgef = sprintf('%s%s.log',path, edgename);

    if exist(edgef, 'file')

        edge = sortByT(stubbornLoad(edgef));
        oi = find(edge(:,1) >= x(i(1),1),1);

        %% plot error
        subplot(2,5,3+M)

        plot(edge(:,1) - edge(1,1),edge(:,[25:26])) %xy img
        
        axis tight

        %% plot measure
        subplot(2,5,8+M)
        
        plot(edge(:,1) - edge(1,1),edge(:,[23:24])) % xy z

        axis tight
        
        Lf = sprintf('%s%s%s.log',path, edgename,'_Lw');
        L = sortByT(stubbornLoad(Lf));
        subplot(2,5,[1 6])       
        
        plot(L(1,3), L(1,4), 'ro')
        axis tight
        
    end
end