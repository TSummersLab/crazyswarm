function PlotDistVer1_0(fileName,plotParam)

t   = plotParam.t;      % Simulation time
n   = plotParam.n;      % Number of agents
q   = plotParam.q;      % Trajectories


%% Folder to save the figure

currentFolder = pwd;
address =  strcat(currentFolder,'/SavedFigs/');


%% Inter-agent distance at each iteration

nItr = size(q,1);
Ds = zeros(n,n, nItr);
for itr = 1 : size(q,1)
    Df = zeros(n,n);
    for i = 1 : n
        for j = i+1 : n
            Df(i,j) = norm(q(itr,3*i-2:3*i)-q(itr,3*j-2:3*j), 2);
        end
    end
    Df = Df + Df';
    Ds(:,:,itr) = Df;    
end


%% Distance Plot

sizeFig = [5.5 3];
position = [2 2, sizeFig];
figure('Units', 'inches', 'Position', position);

% Color map
cmap = [255, 68, 0]./255;

% Plot data
hold on
itr = 0;
for i = 1 : n
    for j = i+1 : n
        itr = itr + 1;
        data = squeeze(Ds(i,j,:));
        plot(t, data, 'Color', cmap);
    end
end
hold off


%% Adjust Font and Axes Properties

hAx = gca;

hXLabel = xlabel('Time (s)','FontWeight','demi' );
hYLabel = ylabel('Distance','FontWeight','demi');

set(hAx                 ,  'FontName',  'Times New Roman');
set([hXLabel, hYLabel]  ,  'FontName',  'Times New Roman');
set(hAx                 ,  'FontSize',  13                );
set([hXLabel, hYLabel]  ,  'FontSize',  12               );

% Set x-y limits
yLim = hAx.YLim;
ylim(hAx, [0, yLim(2)])
xlim(hAx, [t(1), t(end)])

% Adjust axis position in the figure
axPos = get(hAx,'Position');
set(gca,'Position',axPos+[0.01 0.03 0.0 0.0])


%% Save figure

% Save as PDF
fileType = '.pdf';
fullAddress = strcat(address,fileName,fileType);
saveas(gcf,fullAddress)








































































































































































