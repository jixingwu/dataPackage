function hz = computHz(I)

[nrows, ~] = size(I);
dis = zeros(nrows,1);
for i = 1:nrows-1
   dis(i,1) = abs(I(i,1)-I(i+1,1));
end
hz=1e6/mean(dis);
end