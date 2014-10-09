function E = quatToEuler( Q )

% the convention is yzx 

E=zeros(size(Q,1),3);

for i = 1:size(Q,1)
    qw = Q(i,1);
    qx = Q(i,2);
    qy = Q(i,3);
    qz = Q(i,4);

    test = qx*qy + qz*qw;
%     if (test > 0.4999999)  % singularity at north pole
%         heading = 2 * atan2(qx,qw);
%         attitude = pi/2;
%         bank = 0;
% 
%     elseif (test < -0.4999999)  % singularity at south pole
%         heading = -2 * atan2(qx,qw);
%         attitude = - pi/2;
%         bank = 0;
% 
%     else
        sqx = qx*qx;
        sqy = qy*qy;
        sqz = qz*qz;
        heading = atan2(2*qy*qw-2*qx*qz , 1 - 2*sqy - 2*sqz);
        attitude = asin(2*test);
        bank = atan2(2*qx*qw-2*qy*qz , 1 - 2*sqx - 2*sqz);
%     end

    E(i,:) = [heading, attitude, bank];
end

end

