now = datetime;
now.Format = 'yyyy-MM-dd-HH-mm-ss';

if ~exist('system','var')
    system.name = '';
end

filename = strcat(tube_mpc_root, '/', 'data/', strrep(system.name,' ','-'), '- ', string(now));
save(filename)

clear filename now