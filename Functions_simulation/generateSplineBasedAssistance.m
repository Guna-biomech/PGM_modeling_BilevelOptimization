function [dev_moment,nodes_x_all,nodes_y_all] = generateSplineBasedAssistance(nNodes,nodes_x,nodes_y,to_plot)

% check inputs are correct
if nNodes     ==length(nodes_x); else; warning('# x values are not sufficient'); end
if nNodes-2   ==length(nodes_y); else; warning('# y values are not sufficient'); end

% build nodes as absolute values
nodes_x_val=nodes_x;
nodes_x_val(1)  =nodes_x(2)-nodes_x(1);
nodes_x_val(end)=nodes_x(end-1)+nodes_x(end);

nodes_x_abs=[0   nodes_x_val   100];
nodes_y_abs=[0 1 nodes_y     1   0];

nNodes_all =nNodes+2;
nodes_x_all=nodes_x_abs;
nodes_y_all=nodes_y_abs;

% conditions
if nodes_x_abs(2)<=0
    % update nodes in x axis
    nodes_x_all(2)=0;
    nodes_x_all=nodes_x_all(2:end);
    
    % update nodes in y axis
    nodes_y_all=nodes_y_all(2:end);
    nodes_y_all(1)=0;

    % update number of nodes
    nNodes_all=nNodes_all-1;
end

if nodes_x_abs(end-1)>=100
    % update nodes in x axis
    nodes_x_all(end-1)=100;
    nodes_x_all=nodes_x_all(1:end-1);

    % update nodes in y axis
    nodes_y_all=nodes_y_all(1:end-1);
    nodes_y_all(end)=0;

    % update number of nodes
    nNodes_all=nNodes_all-1;
end

% constructing the moment profile
data_length  =100;
stance_phase =linspace(0,100,data_length);

dev_moment = zeros(data_length,1);
y_array    = cell(1,nNodes_all-1); % regions
for iNodes=1:nNodes_all-1
    region = (stance_phase >= nodes_x_all(iNodes)) & (stance_phase <= nodes_x_all(iNodes+1)); % find GC range
    pp     = csape(nodes_x_all(iNodes:iNodes+1), [0, nodes_y_all(iNodes:iNodes+1), 0], 'complete', 'variational');
    y_val  = ppval(pp, stance_phase(region)); % gait_cycle_mesh

    y_array(iNodes)  ={y_val};
    dev_moment(region)=y_val;
end

if to_plot==1
    clf;    hold on;
    plot(stance_phase,dev_moment,'k');
    for iNodes=1:nNodes_all
        plot(nodes_x_all(iNodes),nodes_y_all(iNodes),'.r','MarkerSize',20);
    end
    xlabel('gait cycle [%]');
    ylabel('moment [Nm]');
end
end