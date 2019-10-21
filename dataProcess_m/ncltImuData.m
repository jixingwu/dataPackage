load ncltDataPro;
time_index = max(index);
outFile = zeros(time_index, 10);
outFile(:,1) = inOdometry(1:time_index,1);%timestamp
% outFile(:,2) = inOdometry(1:time_index,5:7)%orientation(w,x,y,z)
% orientation(w,x,y,z)
for i=1:time_index
    omega = deg2rad(inOdometry(i, 5));
    phi = deg2rad(inOdometry(i, 6));
    kappa = deg2rad(inOdometry(i, 7));    
    radians = [omega, phi, kappa];
    outFile(i, 2) = sin(radians(1)/2)*cos(radians(2)/2)*cos(radians(3)/2) - cos(radians(1)/2)*sin(radians(2)/2)*sin(radians(3)/2);%x
    outFile(i, 3) = cos(radians(1)/2)*sin(radians(2)/2)*cos(radians(3)/2) + sin(radians(1)/2)*cos(radians(2)/2)*sin(radians(3)/2);%y
    outFile(i, 4) = cos(radians(1)/2)*cos(radians(2)/2)*sin(radians(3)/2) - sin(radians(1)/2)*sin(radians(2)/2)*cos(radians(3)/2);%z
    outFile(i, 5) = cos(radians(1)/2)*cos(radians(2)/2)*cos(radians(3)/2) + sin(radians(1)/2)*sin(radians(2)/2)*sin(radians(3)/2);%w
end

% %accel
% outFile(:,6) = inMs25(1:time_index,5);
% outFile(:,7) = inMs25(1:time_index,6);
% outFile(:,8) = inMs25(1:time_index,7);
% %angular
% outFile(:,9) = inMs25(1:time_index,8);
% outFile(:,10) = inMs25(1:time_index,9);
% outFile(:,11) = inMs25(1:time_index,10);

%accel
outFile(1:index(1,1),6) = inMs25(1,5);
outFile(1:index(1,1),7) = inMs25(1,6);
outFile(1:index(1,1),8) = inMs25(1,7);
outFile(1:index(1,1),9) = inMs25(1,8);
outFile(1:index(1,1),10) = inMs25(1,9);
outFile(1:index(1,1),11) = inMs25(1,10);
[rows,~] = size(index);
for i = 2:rows
    %accel
    outFile((index(i-1,1)+1):index(i,1),6) = inMs25(i,5);
    outFile((index(i-1,1)+1):index(i,1),7) = inMs25(i,6);
    outFile((index(i-1,1)+1):index(i,1),8) = inMs25(i,7);
    %angular
    outFile((index(i-1,1)+1):index(i,1),9) = inMs25(i,8);
    outFile((index(i-1,1)+1):index(i,1),10) = inMs25(i,9);
    outFile((index(i-1,1)+1):index(i,1),11) = inMs25(i,10);
    
end
writematrix(outFile, '/media/jixingwu/datasetj/NCLT/sensor_data/2012-01-08/imu_100hz.csv', 'Delimiter', ',');


