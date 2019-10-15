clear all;
inFilevio = importdata('/home/jixingwu/vins_output/agz/vio.csv');
[rows_vio,cols_vio] = size(inFilevio);
% outFile = zeros(rows_vio, cols_vio-3);
outFile = inFilevio(:,1:cols_vio-3);
writematrix(outFile, 'Agz_vio.txt', 'Delimiter', ' ');