function[MC_parameter,E_rate,w_rate,h_rate]= MC_BH04_R_prior(muscle_parameter,muscle_DynCon,option,basalOn)
% read inputs
f_FT= muscle_parameter.FT;      % from 0 to 1
OFL = muscle_parameter.OFL;
MCV = muscle_parameter.MCV;
MIF = muscle_parameter.MIF;                 
PCSA= muscle_parameter.PCSA;
MASS= muscle_parameter.mass;

excitation= muscle_DynCon.muscle_excitation;
activation= muscle_DynCon.muscle_activation;
MTUforce  = muscle_DynCon.muscle_MTUforce;     
V_CE      = muscle_DynCon.V_CE; %+shortening -lengthening
W_CE      = muscle_DynCon.W_CE; % W_CE= F_CE*V_CE: +W_CE spent -W_CE gain

lMtilde  = muscle_DynCon.lMtilde;
vMtilde  = muscle_DynCon.vMtilde;
fl_act_multiplier=muscle_DynCon.fl_act_multiplier(:);
f_v_multiplier   =muscle_DynCon.f_v_multiplier(:);
fl_pas_multiplier=muscle_DynCon.fl_pas_multiplier(:);

time=muscle_DynCon.time;
data_length= length(time);
%% PHYSIOLOGICAL PARAMETERS
% rho= 1059.7; % Umberger
% sigma=0.25*10^6; % changed
% muscle_mass= PCSA*rho*muscle_OFL;
f_ST= 1-f_FT;
%% MUSCLE RECRUITMENT
% Equations
u_f   = 1-cos(excitation*pi/2);
u_s   = sin(excitation*pi/2);
%% ACTIVATION HEAT RATE
% Constants
A_f= 133; % w/kg
A_s=  40; % w/kg
Tao_phi=45/1000; 

c=0; 
t_stim=double.empty;
for i=1:data_length
     if excitation(i)>0.1
        c=c+1;
        t_stim(i,1)=c; 
     else
        t_stim(i,1)=0;
        c=0;
     end
end
time_dc_unit=(time(end)-time(1))/(length(time)-1);
t_stim=t_stim*time_dc_unit;
decay_function=double.empty; 
for i=1:data_length
    if t_stim(i)>0
        decay_function(i)=0.06+exp((-t_stim(i).*excitation(i))./Tao_phi);
    else
        decay_function(i)=0;
    end
end
% decay_function=1; %Maarten modification
h_A_rate=decay_function(:).*MASS.*(A_f*f_FT*u_f +A_s*f_ST*u_s);
%% MAINTENANCE HEAT RATE
% Constants
M_f= 111; % w/kg
M_s=  74; % w/kg 
l_M=zeros(data_length,1);
for i=1:data_length
    if lMtilde(i)>=0 && lMtilde(i)<=0.5
       l_M(i,1)= 0.5;
    elseif lMtilde(i)>0.5 && lMtilde(i)<=1
       l_M(i,1)= lMtilde(i);
    elseif lMtilde(i)>1   && lMtilde(i)<=1.5
       l_M(i,1)= -2*lMtilde(i)+3;
    elseif lMtilde(i)>1.5 && lMtilde(i)<=2
       l_M(i,1)= 0.0;
    end
end
h_M_rate=l_M(:).*MASS.*(M_f*f_FT*u_f(:) + M_s*f_ST*u_s(:));
%% SHORTENING/LENGHTENING HEAT RATE
% Assignation
F_CE_only_act= MIF.*activation.*fl_act_multiplier;
F_muscle     = MIF.*(activation.*fl_act_multiplier.*f_v_multiplier+fl_pas_multiplier);

alpha=double.empty;
for i=1:data_length 
    if V_CE(i)<=0 %shortening
        alpha(i,1)= 0.16.*F_CE_only_act(i)+0.18.*F_muscle(i);
    elseif V_CE(i)>0 %lengthening
        alpha(i,1)= 0.157.*F_muscle(i);
    end
end
h_SL_rate= alpha(:).*V_CE(:);
%% WORK RATE
w_rate = W_CE;
%% ENERGY RATE
h_rate= h_A_rate + h_M_rate + h_SL_rate;
E_rate= w_rate + h_rate;
    
if option==0
    % keep it as it was :D
elseif option>=1
    for i=1:data_length
    if E_rate(i)<=0
       h_SL_rate(i)= -h_A_rate(i) - h_M_rate(i) - w_rate(i);
    end
    end
    h_rate= h_A_rate + h_M_rate + h_SL_rate;
    E_rate= h_rate + w_rate;
end

E_value= cumtrapz(time,E_rate);
h_rate_AM=h_A_rate + h_M_rate;
h_rate_SL=h_SL_rate;

MC_parameter(1,:)= E_rate;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_A_rate;
MC_parameter(7,:)= h_M_rate;
MC_parameter(8,:)= h_SL_rate;
MC_parameter(9,:)= E_value;

MC_parameter(10,:)= V_CE;
MC_parameter(11,:)= alpha;
MC_parameter(12,:)= F_CE_only_act;
MC_parameter(13,:)= F_muscle;
end