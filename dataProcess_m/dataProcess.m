clear all;
inFileGt = importdata('/media/jixingwu/datasetj/AGZ/Log Files/GroundTruthAGL.csv');
inFileGm = importdata('/media/jixingwu/datasetj/AGZ/Log Files/GroundTruthAGM.csv');
imgid = inFileGt.data(1:2706,1);
outFileHeader = {'timestamp','x', 'y','z','q_x', 'q_y', 'q_z', 'q_w'};
[row_imgid,~] = size(imgid);
outFile = zeros(row_imgid,8);

for i=1:row_imgid
    
    rows = imgid(i,1);
    outFile(i, 1) = inFileGm.data(rows,1);
    outFile(i,2:4) = inFileGt.data(i, 2:4);
    
    omega = deg2rad(inFileGt.data(i, 5));
    phi = deg2rad(inFileGt.data(i, 6));
    kappa = deg2rad(inFileGt.data(i, 7));    
    radians = [omega, phi, kappa];
    
    outFile(i, 5) = sin(radians(1)/2)*cos(radians(2)/2)*cos(radians(3)/2) - cos(radians(1)/2)*sin(radians(2)/2)*sin(radians(3)/2);%q_x
    outFile(i, 6) = cos(radians(1)/2)*sin(radians(2)/2)*cos(radians(3)/2) + sin(radians(1)/2)*cos(radians(2)/2)*sin(radians(3)/2);
    outFile(i, 7) = cos(radians(1)/2)*cos(radians(2)/2)*sin(radians(3)/2) - sin(radians(1)/2)*sin(radians(2)/2)*cos(radians(3)/2);
    outFile(i, 8) = cos(radians(1)/2)*cos(radians(2)/2)*cos(radians(3)/2) + sin(radians(1)/2)*sin(radians(2)/2)*sin(radians(3)/2);
    
   
end

writematrix(outFile, 'groundtruth.txt');
 
