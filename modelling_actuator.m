%% Modelling actuator
% Design parameters - they are controlled in experiments
exo_k        = 750; % [N/m]
exo_time     = 30;  % [gait cycle %] (when starts to be actived)
exo_duration = 40;  % [gait cycle %] (actuation will be actived from = exo_time+exo_duration)

% Fixed parameters - instrict to the actuator
pos_delta_p = [0.10 -0.02]; % [m] - respect to marker proximal (hip center)
pos_delta_d = [0.10  0.05]; % [m] - respect to marker distal (knee center)
l_slack     = 0.10; % [m]
exo_damping = 10.0; % [N/s]

%% Test - Oscillation motion
clc;

% time and gait cycle
d2r=pi/180; % deg to rad conversion

tstep = 0.01;
time  = 0:tstep:1; % 1 sec => 1 gait cycle
data_length= length(time);
gait_cycle=linspace(0,100,data_length);

% simulate marker trajectories
marker_p= [linspace(0,1,data_length)' ones(data_length,1)]; % [x,y] this is my ref.
ang  = -sin(pi*time)*40 + 30; % [deg] hip angle from -10 to 30
thigh_length= 0.5;            % [m]   thigh/femur is 50 cm
pelvis_offset_ang=-15-90;     % [deg] offset pelvis angle
marker_d= marker_p + thigh_length*[cos((ang+pelvis_offset_ang)*d2r)' sin((ang+pelvis_offset_ang)*d2r)']; % compute distal marker

% compute model actuation
pos_exo_UA= marker_p + pos_delta_p; % [x,y] position of proximal actuator end
pos_exo_LA= marker_d + pos_delta_d; % [x,y] position of distal actuator end

% length of the actuator ends
exo_length=zeros(data_length,1);
for i=1:data_length
    exo_length(i)=norm(pos_exo_UA(i,:)-pos_exo_LA(i,:));
end

% compute moment arm - respect to marker_p
moment_arm = perpendicular_distances(pos_exo_UA, pos_exo_LA, marker_p);


% interesting -------------------------------------------------------------

% control signal generation
control_signal=zeros(data_length,1); % defined
ind =gait_cycle>exo_time & gait_cycle<(exo_time+exo_duration); % active time
control_signal(ind)=1;

% control signal - smoothed, 1st order adopted to mimic slow increase and
% decrease of the pressure that fills the gel muscle actuator

% Parameters
tau = 0.01; %(ms) It must be changed according to actuator properties (EXP required)
y0  = 0;    % Initial condition

% Time span for the solution
tspan = time; % From t = 0 to t = 1
u     = control_signal; 

u_interp = @(t) interp1(time, u, t, 'linear', 'extrap'); % Interpolating function for u(t)
ode = @(t, y) (u_interp(t) - y) / tau; % first order differential eq.
[t_int, y_int] = ode45(ode, tspan, y0); % Solve the ODE

control_signal_ode=y_int;

% compute forces: spring and damping

exo_delta =exo_length-l_slack;
exo_active_delta_len=exo_delta.*control_signal;            % without ode smoothing (not used)
exo_active_delta_len_smooth=exo_delta.*control_signal_ode; % with ode smoothing

exo_active_delta_vel=gradient(exo_active_delta_len_smooth)./gradient(time'); % velocity
spring_force   =exo_k*exo_active_delta_len_smooth; % spring force
damping_force  =-abs(exo_active_delta_vel*exo_damping); % damping force - compute

actuator_force =spring_force+damping_force; % total force

actuator_force_only_pos=zeros(data_length,1);
actuator_force_only_pos(actuator_force>0)=actuator_force(actuator_force>0);

% compute moment
actuator_moment=moment_arm.*actuator_force_only_pos;

clf

subplot(2,2,1)
plot(gait_cycle, u, 'r--', 'DisplayName', 'Input u(t)','LineWidth',2);
hold on;
plot(gait_cycle, y_int, 'b-', 'DisplayName', 'Solution y(t)','LineWidth',2);
xlabel('gait cycle [%]');
ylabel('signal [ ]');
title('Transformation of control_signal to control_signal_ode','Interpreter','none');
legend('Location','northwest');
ylim([0 1.2])
grid on;
set(gca,'FontSize',15)

subplot(2,2,2)
plot(gait_cycle,actuator_force_only_pos,'k','LineWidth',4, 'DisplayName', 'actuator force (pos)');
hold on;
plot(gait_cycle,actuator_force,'color',	"#77AC30",'LineWidth',2, 'DisplayName', 'actuator force');
plot(gait_cycle,spring_force,'color', "r",'LineWidth',2, 'DisplayName', 'spring force');
plot(gait_cycle,damping_force,'b','LineWidth',2, 'DisplayName', 'damping force');
xlabel('gait cycle [%]');
ylabel('force [N]');
title('forces');
legend('Location','northwest');
grid on;
set(gca,'FontSize',15)

subplot(2,2,3)
plot(gait_cycle,moment_arm,'k','LineWidth',4, 'DisplayName', 'actuator moment');
xlabel('gait cycle [%]');
ylabel('length [m]');
title('moment arm distance');
legend('Location','northwest');
grid on;
set(gca,'FontSize',15)

subplot(2,2,4)
plot(gait_cycle,actuator_moment,'k','LineWidth',4, 'DisplayName', 'actuator moment (pos)')
xlabel('gait cycle [%]');
ylabel('moment [Nm]');
title('moment');
legend('Location','northwest');
grid on;
set(gca,'FontSize',15)
%%
% visualization
figure(2); clf;
for i=1:data_length
    subplot(2,2,1)
    plot(marker_p(i,1),marker_p(i,2),'.k','MarkerSize',20)
    hold on
    plot(marker_d(i,1),marker_d(i,2),'.k','MarkerSize',20)
    plot([marker_p(i,1) marker_d(i,1)], [marker_p(i,2) marker_d(i,2)],'k')

    plot(pos_exo_UA(i,1),pos_exo_UA(i,2),'.r','MarkerSize',20)
    plot(pos_exo_LA(i,1),pos_exo_LA(i,2),'.r','MarkerSize',20)

    hold off
    set(gca,'FontSize',15)
    axis([-0.25 1.75 0 1.25])

    subplot(2,2,3)
    plot(gait_cycle,ang,'k')
    hold on
    plot(gait_cycle(i),ang(i),'.k','MarkerSize',20)
    hold off
    set(gca,'FontSize',15)
    axis([0 100 -15 40])
    ylabel('hip angle [deg]')

    subplot(2,2,4)
    plot(gait_cycle,actuator_moment,'k')
    hold on
    plot(gait_cycle(i),actuator_moment(i),'.k','MarkerSize',20)
    hold off
    set(gca,'FontSize',15)
    axis([0 100 -5 max(actuator_moment)*1.2])
    ylabel('moment actuator [Nm]')    

    subplot(2,2,2)
    plot(gait_cycle,actuator_force_only_pos,'k')
    hold on
    plot(gait_cycle(i),actuator_force_only_pos(i),'.k','MarkerSize',20)
    hold off
    set(gca,'FontSize',15)
    axis([0 100 -5 max(actuator_force_only_pos)*1.2])
    ylabel('moment actuator [Nm]')   

    pause(0.001)
end

% plot(time,ang)
function distances = perpendicular_distances(pos_exo_UA, pos_exo_LA, marker_p)
    % Check input dimensions
    if size(pos_exo_UA, 1) ~= size(pos_exo_LA, 1) || ...
       size(pos_exo_UA, 1) ~= size(marker_p, 1)
        error('Input matrices must have the same number of rows.');
    end

    % Preallocate distance array
    distances = zeros(size(marker_p, 1), 1);

    % Compute the distances
    for i = 1:size(pos_exo_UA, 1)
        % Get points
        A = pos_exo_UA(i, :);
        B = pos_exo_LA(i, :);
        P = marker_p(i, :);
        
        % Direction vector of the line AB
        AB = B - A;
        
        % Vector from A to the marker point P
        AP = P - A;
        
        % Project point P onto line AB
        proj_length = dot(AP, AB) / norm(AB);
        proj_point = A + proj_length * (AB / norm(AB));
        
        % Calculate the distance from marker_p to the projection point
        distances(i) = norm(P - proj_point);
    end
end
