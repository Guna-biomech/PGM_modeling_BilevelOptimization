function [PGMinfo,PGM_force,PGM_moment,gait_cycle]        = PGMactuation_force(PGMinfo,time_series,extra_frame,opt_visual)

data_length =length(time_series);

% conversion to gait cycle
data_GC      = data_length-2*extra_frame;
frames_per_GC= 100/(data_GC-1);
gait_cycle   = 0-frames_per_GC*extra_frame:frames_per_GC:100+frames_per_GC*extra_frame; % case without extra frame-> gait_cycle =linspace(0,100,length(time_series));

% initialize variables
nPGMs= length(PGMinfo.names);
nDOFs= size(PGMinfo.geometry.moment_arm,2); % # degrees of freedom

PGM_force =zeros(nPGMs,data_length);
PGM_moment=zeros(nPGMs,nDOFs,data_length);
timeOnset =zeros(nPGMs,2);

% apply constraints
if PGMinfo.constraint.actuationLim==1
    gaitCycleLim=97;
    for iPGM=1:nPGMs
        actuation_endTiming=PGMinfo.torque.startTime(iPGM)+PGMinfo.torque.duration(iPGM);
        if actuation_endTiming>gaitCycleLim
            PGMinfo.torque.duration(iPGM)=gaitCycleLim-PGMinfo.torque.startTime(iPGM);
            PGMinfo.constraint.flag(iPGM)=1; % constraint flag, duration is recomputed
        else
            PGMinfo.constraint.flag(iPGM)=0;
        end
    end
end

for iPGM=1:nPGMs

    % control signal generation
    control_signal=zeros(data_length,1); % defined
    ind =gait_cycle>PGMinfo.torque.startTime(iPGM) & gait_cycle<(PGMinfo.torque.startTime(iPGM)+PGMinfo.torque.duration(iPGM)); % active time
    control_signal(ind)=1;

    time_selected= time_series(ind);
    timeOnset(iPGM,1)=time_selected(1)-0.01;
    timeOnset(iPGM,2)=time_selected(end);
    % control signal - smoothed, 1st order adopted to mimic slow increase and
    % decrease of the pressure that fills the gel muscle actuator

    % Parameters
    tau = PGMinfo.intrinsic.time_lag(iPGM); % [s] reflects instrisic actuator delay
    y0  = 0;                         % initial condition

    % Time span for the solution
    tspan = time_series; % From t = 0 to t = 1
    u     = control_signal;

    u_interp = @(t) interp1(time_series, u, t, 'linear', 'extrap'); % Interpolating function for u(t)
    ode = @(t, y) (u_interp(t) - y) / tau;                          % first order differential eq.
    [~, y_int] = ode45(ode, tspan, y0);                             % Solve the ODE, gives Time and Y axis

    control_signal_ode=y_int;

    % compute forces: spring and damping
    LDelta             = PGMinfo.geometry.length(:,iPGM)-PGMinfo.intrinsic.slack_length(iPGM);
    % exo_active_delta_len = exo_delta.*control_signal;            % without ode smoothing (not used)
    active_LDeltaSmooth= LDelta.*control_signal_ode;        % with ode smoothing

    % exo_active_delta_vel=gradient(exo_active_delta_len_smooth)./gradient(time'); % velocity
    spring_force   =PGMinfo.torque.stiffness(iPGM)*active_LDeltaSmooth; % spring force
    % damping_force  =-abs(exo_active_delta_vel*exo_damping); % damping force - compute

    total_force =spring_force; %+damping_force; % total force

    total_force_pos=zeros(data_length,1);
    total_force_pos(total_force>0)=total_force(total_force>0);

    PGM_force(iPGM,:)=total_force_pos;

    for iDOF=1:nDOFs
        PGM_moment(iPGM,iDOF,:)=PGMinfo.geometry.moment_arm(:,iDOF,iPGM).*PGM_force(iPGM,:)';
    end
end

if opt_visual ==1
    figure;

    PGM_L_min=min(PGMinfo.geometry.length,[],'all');      PGM_L_max=max(PGMinfo.geometry.length,[],'all');
    PGM_MA_min=min(PGMinfo.geometry.moment_arm,[],'all'); PGM_MA_max=max(PGMinfo.geometry.moment_arm,[],'all');
    
    PGM_force_max =max(PGM_force,[],'all');
    PGM_moment_min=min(PGM_moment,[],'all'); PGM_moment_max=max(PGM_moment,[],'all');
    for iPGM=1:nPGMs
        % % length
        % PGMparam_L=PGMinfo.geometry.length(:,iPGM);
        % PGMparam_Lmin=min(PGMparam_L);
        % PGMparam_Lmax=max(PGMparam_L);
        % PGMparam_Ltot=PGMparam_Lmax - PGMparam_Lmin;
        % 
        % subplot(2,2,1)
        % ind_actuator=find(ind);
        % hold on;
        % plot(time_series,PGMparam_L)
        % yline(PGMinfo.actuator.slack_length,'--r')
        % xline(time_series(ind_actuator(1)),':k')
        % xline(time_series(ind_actuator(end)),':k')
        % % axis([time_series(1) time_series(end) PGMparam_length_min-0.1*PGMparam_length_tot PGMparam_length_max+0.1*PGMparam_length_tot])
        % xlim([time_series(1) time_series(end)])
        % xlabel('time [s]'); ylabel('length [m]');
        % 
        % % moment arm
        % PGMparam_L=PGMinfo.geometry.moment_arm;
        % PGMparam_Lmin=min(PGMparam_L);
        % PGMparam_Lmax=max(PGMparam_L);
        % % PGMparam_length_tot=PGMparam_length_max - PGMparam_length_min;
        % 
        % subplot(2,2,2)
        % plot(time_series,PGMparam_L)
        % xline(time_series(ind_actuator(1)),':k')
        % xline(time_series(ind_actuator(end)),':k')
        % axis([time_series(1) time_series(end) PGMparam_Lmin-0.1*PGMparam_Ltot PGMparam_Lmax+0.1*PGMparam_Ltot])
        % xlabel('time [s]'); ylabel('moment arm [m]');

        subplot(nPGMs,4,1+4*(iPGM-1))
        hold on;
        plot(time_series,PGMinfo.geometry.length(:,iPGM))
        xline(timeOnset(iPGM,1),':k'); xline(timeOnset(iPGM,2),':k'); 
        axis([time_series(1) time_series(end) PGM_L_min PGM_L_max])
        xlabel('time [s]'); ylabel('length [m]'); title(PGMinfo.names{iPGM})        

        subplot(nPGMs,4,2+4*(iPGM-1))
        hold on;    
        for iDOF=1:nDOFs
            plot(time_series,PGMinfo.geometry.moment_arm(:,iDOF,iPGM))
        end
        xline(timeOnset(iPGM,1),':k'); xline(timeOnset(iPGM,2),':k'); 
        legend(PGMinfo.DOFs,'Interpreter','none'); legend boxoff
        axis([time_series(1) time_series(end) PGM_MA_min PGM_MA_max])
        xlabel('time [s]'); ylabel('moment arm [m]'); title(PGMinfo.names{iPGM})

        subplot(nPGMs,4,3+4*(iPGM-1))
        hold on;
        plot(time_series,PGM_force(iPGM,:))
        xline(timeOnset(iPGM,1),':k'); xline(timeOnset(iPGM,2),':k'); 
        axis([time_series(1) time_series(end) 0 1.1*PGM_force_max])
        xlabel('time [s]'); ylabel('force [N]'); title(PGMinfo.names{iPGM})

        subplot(nPGMs,4,4+4*(iPGM-1))
        hold on;
        for iDOF=1:nDOFs
            plot(time_series,squeeze(PGM_moment(iPGM,iDOF,:)))
        end
        xline(timeOnset(iPGM,1),':k'); xline(timeOnset(iPGM,2),':k'); 
        legend(PGMinfo.DOFs,'Interpreter','none'); legend boxoff
        axis([time_series(1) time_series(end) PGM_moment_min PGM_moment_max])
        xlabel('time [s]'); ylabel('moment [Nm]'); title(PGMinfo.names{iPGM})
    end

end
end