clear all;
close all;
clc;
addpath ../../NN_numpy/npy-matlab
M = cell(7,1);
for i=0:1:7
    str=strcat('array_',num2str(i),'.npy');
    M{i+1} = readNPY(str);
end
save NN.mat M