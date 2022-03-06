addpath('./models/')
addpath('./plotting/')
addpath('./examples/')
addpath(genpath('../tbxmanager/')) % tbxmanager needed for set geometry 
tube_mpc_root = pwd;
if exist('./data', 'dir') ~= 7
    mkdir('./data')
end
