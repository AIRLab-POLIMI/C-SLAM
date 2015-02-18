function v2 = rotatevector(v, q)

v2 = zeros(size(q,1),3);

for i = 1:size(q,1)
   v2(i,:) = (quatrot(q(i,:))*v(i,:)')';
end

end

