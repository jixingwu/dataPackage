inMs25 = importdata('/media/jixingwu/datasetj/NCLT/sensor_data/2012-01-08/ms25.csv');
inOdometry = importdata('/media/jixingwu/datasetj/NCLT/sensor_data/2012-01-08/odometry_mu_100hz.csv');
% plot(inOdometry(:,2), inOdometry(:,3));
% outFile
[nrows_ms25,~] = size(inMs25);
[nrows_odo,~] = size(inOdometry);

for i = 1:nrows_ms25
   timestamp = inMs25(i,1);
   ind = find(inOdometry(:,1)<=timestamp);
   k = max(ind)
   
   if k+1>nrows_odo
       break;
   end
   
   if abs(inOdometry(k,1)-timestamp) <= abs(inOdometry(k+1,1)-timestamp)
       index(i,1) = k;
   else
       index(i,1) = k+1;
   end  
end

