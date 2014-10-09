function plotStuff()

global path

global x Sa edge augs 


%% load stuff

posef = sprintf('%s%s',path, 'PoseSE3(W).log');
x = sortByT(stubbornLoad(posef));

%% -------------------- FIGURA 1

%% plot trajectory

set(0,'CurrentFigure',1)
clf
hold on

i = find(x(:,2) == max(x(:,2)));

plot3(x(i,3),x(i,4),x(i,5),'m');
plot3(x(1:i(1),3),x(1:i(1),4),x(1:i(1),5),'k');

for j = i(1:20:length(i))'    
%for j = [size(x,1):-1:1 1]
    plotAxis(x(j,3:5)',x(j,6:9),1)    
end    

grid on
%axis image
axis manual
axis equal
axis([-4 4 -4 4])

%% -------------------- FIGURA 2


set(0,'CurrentFigure',2)
clf

edgename = 'IMUintegralDeltaP';

edgef = sprintf('%s%s.log',path, edgename);

if exist(edgef, 'file')
    
    edge = sortByT(stubbornLoad(edgef));
    oi = find(edge(:,1) >= x(i(1),1),1);
    
    axis tight    
    %% plot error
    subplot(2,5,1)
    
    %plot(edge(oi:end,1),edge(oi:end,25:27))    
    plot(edge(:,1) - edge(1,1),edge(:,50:52))

    axis tight

    %% plot measure
    subplot(2,5,6)

    %plot(edge(oi:end,1),[edge(oi:end,22:24)])
    plot(edge(:,1) - edge(1,1),[edge(:,23:25)])
    
    axis tight
end

%% -------------------- LANDMARK

for M = 0:8

    edgename = sprintf('Track_%d', M);

    edgef = sprintf('%s%s.log',path, edgename);

    if exist(edgef, 'file')

        edge = sortByT(stubbornLoad(edgef));
        oi = find(edge(:,1) >= x(i(1),1),1);

        if M < 4 
        %% plot error
        subplot(2,5,2+M)

        plot(edge(:,1) - edge(1,1),edge(:,[25:26])) %xy img
        
        axis tight
        
        %% plot measure
        subplot(2,5,7+M)
        
        plot(edge(:,1) - edge(1,1),edge(:,[23:24])) % xy z

        axis tight
        end
        %% plot landmarks on map
        
        set(0,'CurrentFigure',1)
        
        Lf = sprintf('%s%s%s.log',path, edgename,'_Lw');
        L = sortByT(stubbornLoad(Lf));

        hold on
        plot(L(1,3), L(1,4), 'ro')
        text(L(1,3), L(1,4), sprintf('L%d', M));
        %axis tight
        
        set(0,'CurrentFigure',2)
        
    end
end