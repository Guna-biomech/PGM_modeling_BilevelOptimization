function[MC_parameter,E_rate,w_rate,h_rate]= MC_BH04_R(musT_param,musE_param,mus_Dynamics,option,basalOn)
% read inputs
FMo=musT_param(1); lMo=musT_param(2); vMmax=musT_param(5);
Mvol=musE_param(1); Mpcsa=musE_param(2); Mft=musE_param(3); Mmass=musE_param(4);
  
time       =mus_Dynamics(1,:);     FMvtilde  =mus_Dynamics(7,:);
MExcitation=mus_Dynamics(2,:);     TForce    =mus_Dynamics(8,:);
MActivation=mus_Dynamics(3,:);     FMlpas    =mus_Dynamics(9,:);
lMtildeopt =mus_Dynamics(4,:);     Fce       =mus_Dynamics(10,:);
vMtilde    =mus_Dynamics(5,:);     Vce       =mus_Dynamics(11,:);
FMltilde   =mus_Dynamics(6,:);     Wdotce    =mus_Dynamics(12,:);

data_length= length(time);
%% PHYSIOLOGICAL PARAMETERS
% rho= 1059.7; % Umberger
% sigma=0.25*10^6; % changed
% muscle_mass= PCSA*rho*muscle_OFL;
Mft  = Mft.*ones(data_length,1);
f_ST  = (1-Mft);
f_active_ST = f_ST;
f_active_FT = Mft;
%% MUSCLE RECRUITMENT FROM Uchida et al. 2016
u_f   = 1-cos(MExcitation*pi/2)';
u_s   = sin(MExcitation*pi/2)';
if option==2
% u_f   = 1-cos(excitation*pi/2);
% u_s   = sin(excitation*pi/2);    
f_active_ST = zeros(data_length,1); % 0-1
for i=1:data_length
    if MExcitation(i)==0
    f_active_ST(i,1)=1;
    else
    f_active_ST(i,1)= f_ST(i)*u_s(i)./(f_ST(i).*u_s(i)+Mft(i).*u_f(i));   
    end
end
f_active_FT=1-f_active_ST;
end
%% ACTIVATION/MAINTENANCE HEAT RATE
% ACTIVATION HEAT RATE
Tao_phi=45/1000;
c=0; 
t_stim=double.empty;
for i=1:data_length
     if MExcitation(i)>0.100
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
        decay_function(i,1)=0.06+exp((-t_stim(i).*MExcitation(i))./Tao_phi);
    else
        decay_function(i,1)=0;
    end
end
decay_function = ones(data_length,1); % simplify evaluation

A_s =  40; % w/kg
A_f = 133; % w/kg
h_A_rate= decay_function(:).*Mmass.*(A_f*f_active_FT(i)*u_f +A_s*f_active_ST(i)*u_s);
% MAINTENANCE HEAT RATE
l_M    =zeros(data_length,1);
for i=1:data_length
    if lMtildeopt(i)>=0 && lMtildeopt(i)<=0.5
        l_M(i,1)= 0.5;
    elseif lMtildeopt(i)>0.5 && lMtildeopt(i)<=1
        l_M(i,1)= lMtildeopt(i);
    elseif lMtildeopt(i)>1   && lMtildeopt(i)<=1.5
        l_M(i,1)= -2*lMtildeopt(i)+3; % l_M(i,1)= -1.44*lMtilde(i)+2.44; %-Yamada 2017
    elseif lMtildeopt(i)>1.5 && lMtildeopt(i)<=2
        l_M(i,1)= 0.0;
    end
end

% l_M(:)=lMtildeopt; % simplify evaluation
M_s=  74; % w/kg 
M_f= 111; % w/kg
h_M_rate=l_M(:).*Mmass.*(M_f*f_active_FT(i)*u_f(:) + M_s*f_active_ST(i)*u_s(:));
%% SHORTENING/LENGTHENING HEAT RATE
F_CE_only_act= FMo.*MActivation.*FMltilde; % activation(:)
F_muscle     = FMo.*(MActivation.*FMltilde.*FMvtilde+FMlpas);
alpha=double.empty;
for i=1:data_length 
    if Vce(i)<=0 %shortening -
        alpha(i,1)= 0.16.*F_CE_only_act(i)+0.18.*F_muscle(i);
    elseif Vce(i)>0 %lengthening +
        alpha(i,1)= 0.157.*F_muscle(i); % this is negative
    end
end
h_SL_rate= -alpha(:).*Vce(:);
%% WORK RATE
w_rate = Wdotce';
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

h_rate_AM=h_A_rate + h_M_rate;
h_rate_SL=h_SL_rate;

if basalOn==1
    h_rate=h_rate+1.2*Mmass; %1.2 W/kg
    E_rate=h_rate + w_rate;
end

totalE = cumtrapz(time,E_rate);

MC_parameter(1,:)= E_rate;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_A_rate;
MC_parameter(7,:)= h_M_rate;
MC_parameter(8,:)= h_SL_rate;
MC_parameter(9,:)= totalE;

MC_parameter(10,:)= Vce;
MC_parameter(11,:)= alpha;
MC_parameter(12,:)= F_CE_only_act;
MC_parameter(13,:)= F_muscle;
end