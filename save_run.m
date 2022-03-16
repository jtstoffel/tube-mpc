now = datetime;
now.Format = 'yyyy-MM-dd-HH-mm-ss';

if ~exist('system','var')
    system.name = '';
end

filename = strcat(pwd, '/', 'data/', strrep(system.name,' ','-'), '- ', string(now));
save(filename)

clear filename now