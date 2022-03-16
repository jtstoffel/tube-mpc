clear; clc
addpath('./models/')
addpath('./plotting/')
addpath('./examples/')
addpath('./figures/')

if exist('./data', 'dir') ~= 7
    mkdir('./data')
end
addpath('./data/')

% tbxmanager needed for set geometry (CHANGE PATH IF NEEDED)
addpath(genpath('../tbxmanager/'))

