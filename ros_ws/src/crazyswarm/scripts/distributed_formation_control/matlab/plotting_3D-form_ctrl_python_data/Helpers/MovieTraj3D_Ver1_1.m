%% Make movie of agents' trajectories
%
%
function MovieTraj3D_Ver1_1(fileName, plotCtrl, plotParam)

N   = plotParam.n;         % Number of agents
q   = plotParam.q;         % Trajectories
adj = plotParam.adj;       % Adjacency matrix
ctrl = plotParam.ctrl;     % Control signal
safety = plotParam.safety; % Safety net data
timestamp = plotParam.t;   % Timestamps
stopFlag = plotParam.stopFlag; % stop flag
permutations = plotParam.permutations;

%% Folder to save the figures

currentFolder   = pwd;
address         =  strcat(currentFolder,'/SavedFigs/');

% Name and address of the video file
fileType        = '.avi';
fullAddress     = strcat(address,fileName,fileType);

% Video settings
vid             = VideoWriter(fullAddress);
vid.Quality     = 100;                      % A value between 0 to 100
% vid.FrameRate   = 20;
vid.FrameRate   = size(timestamp,1)/(timestamp(end) - timestamp(1));


%% Parameters

trace       = 20;                               % Trace length of loci
cmap        = repmat([255, 150, 0]./255, N,1);   % Color map
blobSize0   = 200;                              % Size of initil position circle
blobSize    = 700;                              % Size of agents on the plot


%% Rearrrange q

itrTot = size(q,1);
I3 = eye(3);
for itr = 1 : itrTot
    E = reshape(permutations(itr,:), N, N)';
    qitr = q(itr,:).';
    q(itr,:) = (kron(E',I3) * qitr).';
end



%% Plot data

sizeFig     = [10 8];
position    = [2 2, sizeFig];
figure('Units', 'inches', 'Position', position);
axis equal
box on


% Adjust axis position in the figure
axPos = get(gca,'Position');
set(gca,'Position',axPos+[0.0 0.04 0.0 0.0])

% Video settings
set(gcf,'Renderer','zbuffer');

% Coordinates
X           = q(:,1:3:(3*N-2));           
Y           = q(:,2:3:(3*N-1));
Z           = q(:,3:3:(3*N));

itrTot      = size(X,1);                        % Number of iterations (movie frames)

% Control Vectors
X_ctrl      = ctrl(1:itrTot, 1:3:(3*N-2));
Y_ctrl      = ctrl(1:itrTot, 2:3:(3*N-1));
Z_ctrl      = ctrl(1:itrTot, 3:3:(3*N));

% Safety Net Data
outside      = safety(1:itrTot, 1:2:(2*N-1));
intersection = safety(1:itrTot, 2:2:(2*N));

% x,y,z margins in the figure
xMargin     = 0.3;
yMargin     = 0.3;
zMargin     = 0.3;

% Adjust axis margins 
minX        = min(min(X(:))) - xMargin;             
maxX        = max(max(X(:))) + xMargin;
minY        = min(min(Y(:))) - yMargin;             
maxY        = max(max(Y(:))) + yMargin;
minZ        = min(min(Z(:))) - zMargin;             
maxZ        = max(max(Z(:))) + zMargin;
dX          = maxX - minX;
dY          = maxY - minY;
dZ          = maxZ - minZ;
mX          = mean([minX,maxX]);
mY          = mean([minY,maxY]);
mZ          = mean([minZ,maxZ]);
dXYZ        = max([dX,dY,dZ])/2;
axis([mX-dXYZ   mX+dXYZ   mY-dXYZ   mY+dXYZ   mZ-dXYZ   mZ+dXYZ])


%% Make movie

% Start recording
open(vid);

for itr = 1 : itrTot
    itr
    hold on
    
    % Line transparancy
    alpha       = linspace(0.0,0.95,trace);           % Line transparancy
    alpha       = alpha(max(1,trace-itr):trace);    
    
%     % Plot initial positions as circles
%     if (itr ~= 1) && (itr < trace)
%         for i = 1 : N    
%             scatter3(X(1,i),Y(1,i),Z(1,i), blobSize0,cmap(i,:),...
%                 'MarkerEdgeColor',cmap(i,:).^(alpha(1)), 'LineWidth',2);
%         end
%     end
    
%     % Plot trajectories
%     for i = 1 : N
%         jMin = max(1,itr-trace);
%         for j = jMin : itr-1
%             plot3(X(j:j+1,i),Y(j:j+1,i), Z(j:j+1,i), ...
%                 'Color',cmap(i,:).^(alpha(j-jMin+1)),'LineWidth',3); 
%         end
%     end

    E = reshape(permutations(itr,:), N, N)';
    
    adjt = E'*adj*E;
    % Plot graph adjacency at current location
    for m = 1 : N
        for n = 1 : N
            if adjt(m,n)
            plot3([X(itr,m),X(itr,n)],[Y(itr,m),Y(itr,n)], ...
                 [Z(itr,m),Z(itr,n)], 'LineWidth',2, ...
                 'Color', [0.4 0.4 0.4]);
            end
        end
    end  
    
    % Plot agents and number them
    for i = 1 : N

        % Plot final position
        if outside(itr,i) > 0
            scatter3(X(itr,i),Y(itr,i),Z(itr,i), blobSize, cmap(i,:),...
                'MarkerEdgeColor',[1 0 0], ... % red
                'MarkerFaceColor',[1 0 0], ... %red
                'LineWidth',2.5, 'SizeData', blobSize);
        else if intersection(itr,i) > 0
                scatter3(X(itr,i),Y(itr,i),Z(itr,i), blobSize, cmap(i,:),...
                    'MarkerEdgeColor',[0 0 1], ... %blue
                    'MarkerFaceColor',[0 0 1], ... %blue
                    'LineWidth',2.5, 'SizeData', blobSize);
            else if stopFlag(itr,i) == 1
                    scatter3(X(itr,i),Y(itr,i),Z(itr,i), blobSize, cmap(i,:),...
                        'MarkerEdgeColor',[0 1 0], ... %green
                        'MarkerFaceColor',[0 1 0], ... %green
                        'LineWidth',2.5, 'SizeData', blobSize);
                else
                    scatter3(X(itr,i),Y(itr,i),Z(itr,i), blobSize, cmap(i,:),...
                        'MarkerEdgeColor',cmap(i,:), ...
                        'MarkerFaceColor',cmap(i,:), ...
                        'LineWidth',2.5, 'SizeData', blobSize);
                end
            end
        end
        
        % Number agents at their current locations
        strNums = strtrim(cellstr(num2str(i,'%d')));
        text(X(itr,i),Y(itr,i),Z(itr,i), strNums, ...
            'color', [1,1,1],                    ...
            'VerticalAlignment','middle',        ...
            'HorizontalAlignment','center',      ...
            'FontWeight','demi',  'FontSize',30, ...
            'FontName', 'Times New Roman'          );

    end
    
    if (plotCtrl == true)
        % Plot agents' control commands
        for i = 1 : N

            % current position
            x = X(itr,i);
            y = Y(itr,i);
            z = Z(itr,i);
            p = [x,y,z];

            % control input
            x_c = X_ctrl(itr, i);
            y_c = Y_ctrl(itr, i);
            z_c = Z_ctrl(itr, i);
            c = [x_c, y_c, z_c];

            % cntl direction
            scale = 10.0;
            p2 = p + c*scale;
            x2 = p2(1);
            y2 = p2(2);
            z2 = p2(3);

            plot3([x2, x],[y2, y], [z2, z], ...
                    'LineWidth',5, ...
                    'Color', [(1.0/N)*i 1.0-(1.0/N)*i 1.0]);
        end
    end
    
%     if itr == 7
%         keyboard
%     end
    
%     if (itr >1) && ( any(E(:)-Em(:)) )
%         keyboard;
%     end
    
    Em = E;
    
    %%
    
    % Adjust axis margins    
    axis([mX-dXYZ   mX+dXYZ   mY-dXYZ   mY+dXYZ   mZ-dXYZ   mZ+dXYZ]) % Manual axis limits

    % Axis labels
    hXLabel = xlabel('x','FontWeight','demi');
    hYLabel = ylabel('y','FontWeight','demi');
    hZLabel = zlabel('z','FontWeight','demi');
    
    % Adjust Font and Axes Properties
    hAx = gca;
    set([hAx]                            , 'FontSize'   , 21                );
    set([hXLabel, hYLabel, hZLabel]      , 'FontSize'   , 22                );

    set(hAx                      , ...
   'XGrid'       , 'on'          , ...
   'YGrid'       , 'on'          , ...
   'ZGrid'       , 'on'          , ...
   'LineWidth'   , 1.5            );

    hold off
    drawnow
    
    % 3D view
%     view(hAx,   [-58 0]); 
    view(hAx,  [-90, 90]);
%     view(hAx,   [-0.6, 2.5, 1.3]);
    
    % Write video frame
    writeVideo(vid, getframe(gcf));
    
    if itr ~= itrTot(end), cla, end  % Do not wipe the last frame
end

% Stop recording
close(vid);
























































