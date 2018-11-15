%% add paths

addpath('/home/tylersummers/ssh_cs_ws_3/crazyswarm/ros_ws/src/crazyswarm/scripts/3D_formation_control_matlab/plotting_3D-form_ctrl_python_data/Helpers')
addpath('/home/tylersummers/ssh_cs_ws_3/crazyswarm/ros_ws/src/crazyswarm/scripts/3D_formation_control_matlab/plotting_3D-form_ctrl_python_data/SavedFigs')

%% add files
path_to_data_file = '/home/tylersummers/ssh_cs_ws_3/crazyswarm/ros_ws/src/crazyswarm/scripts/form_ctrl_saved_data/';
data_file = 'ICRA_experiments/good_pyramid/';
time = 'September_09_2018_at_16_hr_26_min_36_sec';
time_and_format = strcat(time, '.csv');
path_to_data = strcat(path_to_data_file, data_file);

%% Initializing variables

% file names:
ctrl_dist_file = strcat(path_to_data, 'control_distance', time_and_format);
ctrl_vel_file = strcat(path_to_data, 'control_velocity', time_and_format);
des_pos_file = strcat(path_to_data, 'desired_position', time_and_format);
cur_pos_file = strcat(path_to_data, 'positions', time_and_format);
des_pos_safe_file = strcat(path_to_data, 'safe_desired_position', time_and_format);
safety_net_file = strcat(path_to_data, 'safet_net_data', time_and_format);
timestamp_file = strcat(path_to_data, 'time_stamp', time_and_format);
stopFlag_file = strcat(path_to_data, 'stopFlag', time_and_format);
stopFlag_old_file = strcat(path_to_data, 'stopFlag_old', time_and_format);
permutations_file = strcat(path_to_data, 'permutations', time_and_format);
colIdx_file = strcat(path_to_data, 'colIdx', time_and_format);

% where to starts
start_row = 1;
start_col = 0;

% formation type
% One of: Tetrahedron, Square-base-pyramid, Pentagon-base-pyramid, Cube,
% 8-agent-struct-1, 8-agent-struct-2, 12-agent-struct-1, 16-agent-scifi
formation = 'Square-base-pyramid';

% file name when saving data
save_file_name = '5-agent';


%% Import data

ctrl_dist = csvread(ctrl_dist_file, start_row, start_col);
ctrl_vel = csvread(ctrl_vel_file, start_row, start_col);
des_pos = csvread(des_pos_file, start_row, start_col);
cur_pos = csvread(cur_pos_file, start_row, start_col);
des_pos_safe = csvread(des_pos_safe_file, start_row, start_col);
safety_net_data = csvread(safety_net_file, start_row, start_col);
timestamp = csvread(timestamp_file, start_row, start_col);
stopFlag = csvread(stopFlag_file, start_row, start_col);
stopFlag_old = csvread(stopFlag_old_file, start_row, start_col);
permutations = csvread(permutations_file, start_row, start_col);
% colIdx = csvread(colIdx_file, start_row, start_col);



%% Prepare for plotting

[rows, cols] = size(des_pos_safe);
% num_itr = rows-1; % take all but last row (it may be incomplete)
num_itr = floor(rows*48/48); % take only a portion of the video
num_rob = cols / 3; % number of agents
n = num_rob;


ctrl_dist = ctrl_dist(1:num_itr, :);
ctrl_vel = ctrl_vel(1:num_itr, :);
des_pos = des_pos(1:num_itr, :);
cur_pos = cur_pos(1:num_itr, :);
des_pos_safe = des_pos_safe(1:num_itr, :);
safety_net_data = safety_net_data(1:num_itr, :);
timestamp = timestamp(1:num_itr, :);
stopFlag = stopFlag(1:num_itr, :);
stopFlag_old = stopFlag_old(1:num_itr, :);
permutations =permutations(1:num_itr, :);
% colIdx = colIdx(1:n*num_itr, :);


[~, qDes, adj, Dd, N] = LoadData(formation);

if N ~= n
    error('Invalid formation for given data')
end

%% Reconstructing E from saved data

E = zeros(N, N, num_itr);
for itr = 1 : num_itr
    E(:,:,itr) = reshape(permutations(itr,:), N, N)';
end

%%

% qDes = [ sqrt(8/9),  0        ,   -1/3;
%        -sqrt(2/9),  sqrt(2/3),   -1/3;
%        -sqrt(2/9), -sqrt(2/3),   -1/3;
%         0        ,  0        ,    .7  ]' .* .5; % tetrahedron

% qDes = [ 0  0  1  1  0  0  1  1; ...
%          0  1  1  0  0  1  1  0; ...
%          0  0  0  0  1  1  1  1]; %cube

% qDes = [ 0  0  1  1  0.5  0.5  1.5  1.5; ...
%          0  1  1  0  0.5  1.5  1.5  0.5; ...
%          0  0  0  0   1    1    1    1]; %scube

% qDes = [0.9511, 0.3090, 0.0;...
%         0.5878, -0.8090, 0.0;...
%         -0.5878, -0.8090, 0.0;...
%         -0.9511, 0.3090, 0.0;...
%         -0.0000, 1.0000, 0.0;...
%         0.0, 0.0, 1.0]'.* 0.65; % penta-base pyramid
% 
% qDes = [0.9511, 0.3090, 0.0;...
%         0.5878, -0.8090, 0.0;...
%         -0.5878, -0.8090, 0.0;...
%         0.0, 0.0, 1.0; ...
%         -0.9511, 0.3090, 0.0;...
%         -0.0000, 1.0000, 0.0]'.* 0.65; % penta-base pyramid

% qDes = [ ...
% -0.7500         0         0;...
%     0.7500         0         0;...
%          0   -0.7500         0;...
%          0    0.7500         0;...
%    -0.3000    0.3000    0.3000;...
%     0.3000    0.3000    0.3000;...
%     0.3000   -0.3000    0.3000;...
%    -0.3000   -0.3000    0.3000;...
%    -0.7500    0.7500         0;...
%     0.7500    0.7500         0;...
%     0.7500   -0.7500         0;...
%    -0.7500   -0.7500         0]'; %12-agent

%  qDes = [-0.5,  0.0,   0.0; ...
%          -0.5, -1.0,   0.0; ...
%           0.5, -1.0,   0.0; ...
%           0.5,  0.0,   0.0; ...
%           0.0, -0.5,   0.7]'; % pyramid


% qDes = [ 0   0   0; ...
%           1   0   0; ...
%           1   1   0; ...
%           0   1   0; ...
%           0  0.5 0.5; ...
%          0.5  0  0.5; ...
%           1  0.5 0.5; ...
%          0.5  1  0.5; ]' *1.5; % 8-agent structure
     
     
     
% adj = ones(num_rob, num_rob) - eye(num_rob);
% adj = [ 0     1     1     0     1     0     0     0;
%         1     0     0     1     0     1     0     0;
%         1     0     0     1     0     0     1     0;
%         0     1     1     0     0     0     0     1;
%         1     0     0     0     0     1     1     0;
%         0     1     0     0     1     0     0     1;
%         0     0     1     0     1     0     0     1;
%         0     0     0     1     0     1     1     0]; % cube/scube
%     
% adj=    [0,  1,  0,  0,  1,  1;
%     1,  0,  1,  0,  0,  1;
%     0,  1,  0,  1,  0,  1;
%     0,  0,  1,  0,  1,  1;
%     1,  0,  0,  1,  0,  1;
%     1,  1,  1,  1,  1,  0] % penta-base pyramid
% 
% adj = [...
%   0   0   0   0   0   0   0   0   1   0   0   1;...
%    0   0   0   0   0   0   0   0   0   1   1   0;...
%    0   0   0   0   0   0   0   0   0   0   1   1;...
%    0   0   0   0   0   0   0   0   1   1   0   0;...
%    0   0   0   0   0   1   0   1   1   0   0   0;...
%    0   0   0   0   1   0   1   0   0   1   0   0;...
%    0   0   0   0   0   1   0   1   0   0   1   0;...
%    0   0   0   0   1   0   1   0   0   0   0   1;...
%    1   0   0   1   1   0   0   0   0   1   0   1;...
%    0   1   0   1   0   1   0   0   1   0   1   0;...
%    0   1   1   0   0   0   1   0   0   1   0   1;...
%    1   0   1   0   0   0   0   1   1   0   1   0]; %12 agent

% 
% 
% adj = [ ...
% 0   1   0   1   1   1   0   0    ; ...
% 1   0   1   0   0   1   1   0    ; ...
% 0   1   0   1   0   0   1   1    ; ...
% 1   0   1   0   1   0   0   1    ; ...
% 1   0   0   1   0   1   0   1    ; ...
% 1   1   0   0   1   0   1   0    ; ...
% 0   1   1   0   0   1   0   1    ; ...
% 0   0   1   1   1   0   1   0    ]; % 8 -agent structure 

% % adj for 12 robots
% adjIdx = [1 2 3 4 5 6 7 8  9 10 11 12 1 2 3 4 5  6  7  8 9 10  3  4
%           2 3 4 1 6 7 8 5 10 11 12  9 5 6 7 8 9 10 11 12 1  2 11 12];      
% 
% adj = zeros(n,n);
% for i = 1 : size(adjIdx,2)
%     adj(adjIdx(1,i), adjIdx(2,i)) = 1;
% end
% adj = adj + adj.';
% adj = (adj >= 1);  

%%


close all

plotParam.n = num_rob; % number of agents
plotParam.q = cur_pos; % trajectory
plotParam.ctrl = ctrl_dist; % control input
plotParam.qDes = qDes; % desired formation
plotParam.t     = timestamp; % time stamp
plotParam.adj   = adj; % Adjacency matrix
plotParam.safety = safety_net_data; % safety net data (outside safe zone, intersection)
plotParam.des_pos = des_pos; % calculated desired position
plotParam.stopFlag = stopFlag_old; %stop flag
plotParam.permutations = permutations;


saveFig         = true; % Choose to save figure or not
plotCtrl        = true; % Choose to plot control signal or not
fileName        = strcat(save_file_name, time, ''); % Name of the saved figure


%% Plot

% Plot figure
% PlotTraj3D_Ver1_1(fileName, saveFig, plotParam)

% Make movie + color code stop flag %% TODO
MovieTraj3D_Ver1_1(fileName, plotCtrl, plotParam)

% Plot desired location and current location vs time
% PlotDataVTime_Ver1_1(fileName, saveFig, plotParam)

% Interagent distance vs time
% PlotDistVer1_0(fileName,plotParam)

%% Saveing workspace data
save(fileName)






