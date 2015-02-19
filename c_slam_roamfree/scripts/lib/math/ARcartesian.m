function [ rect ] = ARcartesian( tr, qr, wbar, ff, omega )
%ARCARTESIAN from anchored rectangles to cartesian rectangles
%   Detailed explanation goes here

Mw = [wbar; 0; 0];
Mh = [0; wbar / ff; 0];
M1 = [0; 0; 0];
M2 = M1 + Mw;
M3 = M1 +Mw + Mh;
M4 = M1 + Mh;

M = [M1, M2, M3, M4];

rect = zeros(4, 3);
for i = 1:size(M,2)
    rect(i, :) = (tr + 1/omega*quatrot(qr)*M(:, i))';
end

end

