function plotLandmark( landmark, index, format, color )
%PLOTLANDMARK Plot a single landmark
%   Detailed explanation goes here
M1 = landmark(1:3);
M2 = landmark(4:6);
M3 = landmark(7:9);
M4 = landmark(10:12);

CM = (M1 +M2+M3+M4)/4;

text(CM(1), CM(2), CM(3), strcat('LandMark', int2str(index)), 'HorizontalAlignment', 'Center');

plot3(M1(1), M1(2), M1(3), format);
plot3(M2(1), M2(2), M2(3), format);
plot3(M3(1), M3(2), M3(3), format);
plot3(M4(1), M4(2), M4(3), format);


pts = [M1; M2; M3; M4; M1];
line(pts(:,1), pts(:,2), pts(:,3), 'Color', color);



end

