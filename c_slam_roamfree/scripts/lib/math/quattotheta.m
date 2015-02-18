function theta = quatToTheta(q)

theta = zeros(size(q,2),1);

for i = 1:length(q)
    R = quatrot(q(i,:));
    theta(i) = atan2(R(2,1),R(1,1));    
end

theta = unwrap(theta);

end

