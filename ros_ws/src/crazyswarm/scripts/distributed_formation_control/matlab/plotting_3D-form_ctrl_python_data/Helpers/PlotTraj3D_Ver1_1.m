% Ver 1_1: 
%           - 3D plot of trajectories
%           
%


function PlotTraj3D_Ver1_1(fileName,saveFig,plotParam)

N   = plotParam.n;       % Number of agents
q   = plotParam.q;       % Trajectories
adj = plotParam.adj;     % Adjacency matrix


%% Folder to save the figure

currentFolder   = pwd;
address         = strcat(currentFolder,'/SavedFigs/');


%% Parameters

trace       = size(q,1);                        % Trace length of loci
cmap        = repmat([255, 68, 0]./255, N,1);   % Color map
blobSize0   = 100;                              % Size of initil position circle
blobSize    = 400;                              % Size of agents on the plot

nrmTheresh  = 1e-16;                            % Thereshold for trimming the end of movie
Dq          = diff(q);
DqNrm       = sum(abs(Dq),2) / N;
qShort      = q(DqNrm > nrmTheresh,:);          % Cut the uninteresting end part, which is stationary


%% Plot data

sizeFig     = [5 4];
position    = [2 2, sizeFig];
figure('Units', 'inches', 'Position', position);
axis equal
box on

% Coordinates
X           = qShort(:,1:3:(3*N-2));           
Y           = qShort(:,2:3:(3*N-1));
Z           = qShort(:,3:3:(3*N));

itrTot      = size(X,1);                        % Number of iterations (movie frames)

% x and y margins in the figure
xMargin     = 0.2;
yMargin     = 0.2;
zMargin     = 0.2;

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

hold on
axis([mX-dXYZ   mX+dXYZ   mY-dXYZ   mY+dXYZ   mZ-dXYZ   mZ+dXYZ])



%% Draw plot

% Line transparancy
alpha       = linspace(0.17,1,trace);           % Line transparancy
alpha       = alpha(max(1,trace-itrTot):trace);

% Plot initial positions as circles
for i = 1 : N    
    scatter3(X(1,i),Y(1,i),Z(1,i), blobSize0,cmap(i,:),...
        'MarkerEdgeColor',cmap(i,:).^(alpha(1)), 'LineWidth',2);
end

% Plot trajectories
for i = 1 : N
    jMin = max(1,itrTot-trace);
    for j = jMin : itrTot-1
        plot3(X(j:j+1,i),Y(j:j+1,i), Z(j:j+1,i), ...
            'Color',cmap(i,:).^(alpha(j-jMin+1)),'LineWidth',3); 
    end
end

% Plot graph adjacency at current location
for m = 1 : N
    for n = 1 : N
        if adj(m,n)
        plot3([X(itrTot,m),X(itrTot,n)],[Y(itrTot,m),Y(itrTot,n)], ...
             [Z(itrTot,m),Z(itrTot,n)], 'LineWidth',2, ...
             'Color', [0.4 0.4 0.4]);
        end
    end
end   

% Plot agents and number them
for i = 1 : N
    
    % Plot final position
    scatter3(X(itrTot,i),Y(itrTot,i),Z(itrTot,i), blobSize, cmap(i,:),...
        'MarkerEdgeColor',cmap(i,:),...
        'MarkerFaceColor',cmap(i,:),'LineWidth',2);
    
    % Number agents at their current locations
    strNums = strtrim(cellstr(num2str(i,'%d')));
    text(X(itrTot,i),Y(itrTot,i),Z(itrTot,i), strNums, ...
        'color', [1,1,1],                 ...
        'VerticalAlignment','middle',     ...
        'HorizontalAlignment','center',   ...
        'FontWeight','demi','FontSize',18,...
        'FontName'   , 'Times New Roman'     );
end

hold off


%% Adjust Font and Axes Properties

% Adjust axis margins    
axis([mX-dXYZ   mX+dXYZ   mY-dXYZ   mY+dXYZ   mZ-dXYZ   mZ+dXYZ]) % Manual axis limits

% Axis labels
hXLabel = xlabel('x','FontWeight','demi');
hYLabel = ylabel('y','FontWeight','demi');
hZLabel = zlabel('z','FontWeight','demi');

% Adjust axis position in the figure
axPos = get(gca,'Position');
set(gca,'Position',axPos+[0.0 0.04 0.0 0.0])

% Adjust Font and Axes Properties
hAx = gca;
set([hAx]                            , 'FontSize'   , 13                );
set([hXLabel, hYLabel, hZLabel]      , 'FontSize'   , 14                );

set(hAx                          , ...
   'XGrid'       , 'on'          , ...
   'YGrid'       , 'on'          , ...
   'ZGrid'       , 'on'          , ...
   'LineWidth'   , 1         );

% 3D view
view(hAx         ,  [-50 20]); 


%% Save the Fig

if saveFig
    
    set(gcf, 'PaperPositionMode', 'auto');
    set(gcf, 'PaperUnits', 'inches', 'PaperSize', sizeFig);

    % Get current figure
    hdl = gcf;

    % Save as Figure
    fileType = '.fig';
    fullAddress = strcat(address,fileName,fileType);
    saveas(hdl,fullAddress)

    % Save as PDF
    fileType = '.pdf';
    fullAddress = strcat(address,fileName,fileType);
    saveas(hdl,fullAddress)

end
