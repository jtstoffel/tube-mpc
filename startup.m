clear; clc
addpath('./models/')
addpath('./plotting/')
addpath('./examples/')
addpath('./figures/')
addpath('./utils/')
addpath('./planner/')
addpath('./etoc/')

if exist('./data', 'dir') ~= 7
    mkdir('./data')
end
addpath('./data/')

% tbxmanager needed for set geometry (CHANGE PATH IF NEEDED)
TBX_MANAGER_PATH = '../tbxmanager/';
addpath(genpath(TBX_MANAGER_PATH))

clear TBX_MANAGER_PATH