function[MC_parameter,w_rate,h_rate,E_rate,h_A_rate,h_M_rate,h_SL_rate]= MC_HO06(muscle_parameter,muscle_DynCon,time,option)
% read inputs
f_FT= muscle_parameter.FT;
OFL = muscle_parameter.OFL;
MCV = muscle_parameter.MCV;
MIF = muscle_parameter.MIF;                      % NEEDED
PCSA= muscle_parameter.PCSA;
MASS= muscle_parameter.mass;

excitation= muscle_DynCon.muscle_excitation;
activation= muscle_DynCon.muscle_activation;
MTUforce  = muscle_DynCon.muscle_MTUforce;       % NOT NEEDED

lMtilde  = muscle_DynCon.lMtilde;
vMtilde  = muscle_DynCon.vMtilde;
fl_act_multiplier=muscle_DynCon.fl_act_multiplier;
f_v_multiplier   =muscle_DynCon.f_v_multiplier;
fl_pas_multiplier=muscle_DynCon.fl_pas_multiplier;

data_length= length(time);
%% ACTIVATION HEAT RATE

gDotTilde_FT=52.50; % w/kg
gDotTilde_ST=10.98; % w/kg
k3_FT= 12;
k3_ST= 6;
k4_FT= 14;
k4_ST= 8;

gDotTilde= gDotTilde_FT*(f_FT/100)+gDotTilde_ST*(1-f_FT/100);
k3= k3_FT*(f_FT/100)+k3_ST*(1-f_FT/100);
k4= k4_FT*(f_FT/100)+k4_ST*(1-f_FT/100);

vRelativeStimulationFreq= activation(:).^2;
vMAXFreq= k3+k4.*activation(:);

% Equation
h_A_rate= MASS*gDotTilde.*vRelativeStimulationFreq.*...
           ((1-exp(-0.25-18.2./(vRelativeStimulationFreq.*vMAXFreq)))./...
           (1-exp(-0.25-18.2./vMAXFreq))); 
%% MAINTANCE HEAT RATE

hDotTilde_FT=97.50; % [W/kg]
hDotTilde_ST=13.42; % [W/kg]
hDotTilde= hDotTilde_FT*(f_FT/100)+hDotTilde_ST*(1-f_FT/100);
phiTilde= gDotTilde./(gDotTilde+hDotTilde);

h_M_rate= MASS*(gDotTilde+hDotTilde).*activation(:).*(fl_act_multiplier-phiTilde);
%% Shortening/lengthening heat rate

%muscle_MIF= soleus_MIF;
muscle_fiber_velocity = vMtilde.*MCV*OFL;

aTilde_coefficient_FT=0.28; % [W/kg]
aTilde_coefficient_ST=0.16; % [W/kg]
aTilde_coefficient= aTilde_coefficient_FT*(f_FT/100)+...
                    aTilde_coefficient_ST*(1-f_FT/100);
aTilde= aTilde_coefficient*MIF; % 

for i=1:data_length
   if muscle_fiber_velocity(i)<=0
      h_SL_rate(i,1)= -aTilde.*activation(i).*fl_act_multiplier(i)*(muscle_fiber_velocity(i));
   elseif muscle_fiber_velocity(i)>0
      h_SL_rate(i,1)= 0;
   end
end
%% Work rate
F_CE = MIF.*activation(:).*fl_act_multiplier(:).*f_v_multiplier(:);
w_rate = -muscle_fiber_velocity(:).*F_CE(:);
%%  Energy rate
h_rate= h_A_rate + h_M_rate + h_SL_rate;
E_rate= w_rate + h_rate;
    
if option==0
    % keep it as it was :D
elseif option==1
    for i=1:data_length
    if E_rate(i)<=0
       h_SL_rate(i,1)= -h_A_rate(i) - h_M_rate(i) - w_rate(i);
    end
    end
    h_rate= h_A_rate + h_M_rate + h_SL_rate;
    E_rate= h_rate + w_rate;
end

E_value= cumtrapz(time,E_rate);

MC_parameter(1,:)= E_rate;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate;
MC_parameter(4,:)= h_A_rate;
MC_parameter(5,:)= h_M_rate;
MC_parameter(6,:)= h_SL_rate;
MC_parameter(7,:)= E_value;
end