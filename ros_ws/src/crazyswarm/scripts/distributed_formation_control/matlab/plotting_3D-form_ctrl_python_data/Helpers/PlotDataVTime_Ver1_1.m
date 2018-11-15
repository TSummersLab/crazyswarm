% Ver 1_1:
%           - Plotting the current position, desired position, and control
%           signal versus time
%
%
%

function PlotDataVTime_Ver1_1(file_name, saveFig, plotParam)
%PLOTDATAVTIME_VER1_1 Plots data relative to time
%   Plots the current and desired positions and control signal along the x, y, and z axis versus time

%% Retrieving data
pos_c      = plotParam.q;
pos_d      = plotParam.des_pos;
ctrl_dist  = plotParam.ctrl;
timestamp  = plotParam.t;
num_rob    = plotParam.n;
safety_net = plotParam.safety;


%% Data for saving figure
currentFolder = pwd;
address       = strcat(currentFolder, '/SavedFigs/');


%% Plotting parameters

trace = size(timestamp,1) - 1;            % Trace length of data
timestamp = timestamp(1:trace);
timestamp = linspace(0, timestamp(trace)-timestamp(1), trace);
if trace <=0 
    return
end
cmap = zeros(num_rob, 3);                % Colo map
for i = 1 : num_rob
    cmap(i,1) = 1.0/num_rob * i;
    cmap(i,2) = 1.0 - (1.0/num_rob * i);
    cmap(i,3) = 1.0;
end


%% Plot data
% 
% fx = figure('Name', 'X values', 'Units', 'centimeters');
% subplot(2,1,1)
% box on;
% subplot(2,1,2)
% box on;
% 
% fy = figure('Name', 'Y values', 'Units', 'centimeters');
% subplot(2,1,1)
% box on;
% subplot(2,1,2)
% box on;
% 
% fz = figure('Name', 'Z values', 'Units', 'centimeters');
% subplot(2,1,1)
% box on;
% subplot(2,1,2)
% box on;


% Current position coordinates converted to cm
X_c = pos_c(1:trace, 1:3:(3*num_rob-2))*100;
Y_c = pos_c(1:trace, 2:3:(3*num_rob-1))*100;
Z_c = pos_c(1:trace, 3:3:(3*num_rob-0))*100;


% Desired postion coordinates converted to cm
X_d = pos_d(1:trace, 1:3:(3*num_rob-2))*100;
Y_d = pos_d(1:trace, 2:3:(3*num_rob-1))*100;
Z_d = pos_d(1:trace, 3:3:(3*num_rob-0))*100;


% Calculated control converted to cm
CTRL_X = ctrl_dist(1:trace, 1:3:(3*num_rob-2))*100;
CTRL_Y = ctrl_dist(1:trace, 2:3:(3*num_rob-1))*100;
CTRL_Z = ctrl_dist(1:trace, 3:3:(3*num_rob-0))*100;

outside      = safety_net(1:trace, 1:2:(2*num_rob-1));
intersection = safety_net(1:trace, 2:2:(2*num_rob));

safety = ones(trace,num_rob);
for i=1:trace
    for j = 1:num_rob
        if ((outside(i,j)> 0) | (intersection(i,j)>0))
            safety(i,j) = 0;
        end
    end
end


%% Draw Plot and save data

for i = 1:num_rob
    fig_name = strcat('Data values for agent ', num2str(i));
    
    % Position
    figure('Name', fig_name, 'Units', 'centimeters');
    subplot(3,2,1);
    title(strcat('X position for agent ', num2str(i)));
    hold on;
    plot(timestamp, X_c(:,i), 'Color', [0,0,0], 'LineStyle', '-', ...
         'LineWidth', 0.7); % current 
    plot(timestamp, X_d(:,i), 'Color', cmap(i,:), 'LineStyle', '--', ...
         'LineWidth', 0.7); % desired
    subplot(3,2,2);
    hold on;
    title(strcat('X control for agent ', num2str(i)));
    plot(timestamp, CTRL_X(:,i), 'Color', cmap(i,:), 'LineStyle', '-', ...
         'LineWidth', 0.7); % control
    plot(timestamp, safety(:,i), 'Color', [255,200,200]./255, 'LineWidth', 0.3)

    
    subplot(3,2,3);
    title(strcat('Y position for agent ', num2str(i)));
    hold on;
    plot(timestamp, Y_c(:,i), 'Color', [0,0,0], 'LineStyle', '-', ...
         'LineWidth', 0.7); % current 
    plot(timestamp, Y_d(:,i), 'Color', cmap(i,:), 'LineStyle', '--', ...
         'LineWidth', 0.7); % desired
    subplot(3,2,4);
    hold on;
    title(strcat('Y control for agent ', num2str(i)));
    plot(timestamp, CTRL_Y(:,i), 'Color', cmap(i,:), 'LineStyle', '-', ...
         'LineWidth', 0.7); % control
    plot(timestamp, safety(:,i), 'Color', [255,200,200]./255, 'LineWidth', 0.3)
     
     
    subplot(3,2,5);
    title(strcat('Z position for agent ', num2str(i)));
    hold on;
    plot(timestamp, Z_c(:,i), 'Color', [0,0,0], 'LineStyle', '-', ...
         'LineWidth', 0.7); % current 
    plot(timestamp, Z_d(:,i), 'Color', cmap(i,:), 'LineStyle', '--', ...
         'LineWidth', 0.7); % desired
    subplot(3,2,6);
    hold on;
    title(strcat('Z control for agent ', num2str(i)));
    plot(timestamp, CTRL_Z(:,i), 'Color', cmap(i,:), 'LineStyle', '-', ...
         'LineWidth', 0.7); % control
    plot(timestamp, safety(:,i), 'Color', [255,200,200]./255, 'LineWidth', 0.3)
     
    if saveFig
    
        set(gcf, 'PaperPositionMode', 'auto');
        
        % Get current figure
        hdl = gcf;

        % Save as Figure
        fileType = '.fig';
        fullAddress = strcat(address,file_name,'_',fig_name,fileType);
        saveas(hdl,fullAddress)

        % Save as PDF
        fileType = '.pdf';
        fullAddress = strcat(address,file_name,'_',fig_name,fileType);
        saveas(hdl,fullAddress)
     end 
end








end
