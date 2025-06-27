function[MC_parameter,w_rate,h_rate,E_rate,h_AM_rate,h_SL_rate]= MC_UM03(muscle_parameter,muscle_DynCon,time,option)
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
% F_max calculated, not extracted from MIF
% Commmentaries
% F_max calculated, not extracted from MIF
% S is aerobic
%% Physiological related parameters
% rho= 1059.7;    sigma=0.25*10^6;
S= 1.5; %Aerobic

% f_FT=f_FT*100;
f_FT=f_FT;
f_ST=1-f_FT;
% m= PCSA*rho*muscle_OFL;
%F_max= sigma* PCSA;
F_max= MIF;
%% Scaling factor
muscle_fiber_velocity= vMtilde.*MCV.*OFL;
VCEtilde= muscle_fiber_velocity./OFL;

for i=1:data_length
    if excitation(i) > activation(i)
    A(i,1)= excitation(i);
    elseif excitation(i) <= activation(i)
    A(i,1)= (excitation(i)+activation(i))/2;      
    end
end

A_AM= A.^(0.6);

for i=1:data_length
    if VCEtilde(i) <= 0
    A_S(i,1)= A(i).^2;
    elseif VCEtilde(i) > 0
    A_S(i,1)= A(i);
    end
end 
%% Velocity related parameters
VCE_MAX= MCV;

VCE_MAX_ST= VCE_MAX/2.5;
VCE_MAX_FT= VCE_MAX;

alpha_S_ST= 100/VCE_MAX_ST;
alpha_S_FT= 153/VCE_MAX_FT;
alpha_L= 4*alpha_S_ST;
%% Activation/maintenance heat rate
L_CE= lMtilde*OFL;
h_AM_rate= ones(1,data_length)*1.28*f_FT + 25;

for i=1:data_length
   if L_CE(i)<=OFL
   h_AM_rate_scaled(i,1)= h_AM_rate(i).*A_AM(i).*S*MASS;
   elseif L_CE(i)>OFL
   h_AM_rate_scaled(i,1)= (0.4*h_AM_rate(i)+0.6*h_AM_rate(i)*fl_act_multiplier(i)).*A_AM(i).*S*MASS;
   end
end
%% Shortening/lengthening heat rate
for i=1:data_length
   if VCEtilde(i)<=0
      
       term_conditioned(i,1)=(-alpha_S_ST.*VCEtilde(i).*(1-f_FT/100));
       if term_conditioned(i) >= alpha_S_ST*VCE_MAX_ST
          term_conditioned(i,1) = alpha_S_ST.*VCE_MAX_ST;
       end
       
      h_SL_rate(i,1)= (term_conditioned(i)...
                     -alpha_S_FT.*VCEtilde(i).*(f_FT/100)).*A_S(i);
   elseif VCEtilde(i)>0
      h_SL_rate(i,1)= alpha_L*VCEtilde(i).*A(i);
   end
end

for i=1:data_length
   if L_CE(i)<=OFL
   h_SL_rate_scaled(i,1)= h_SL_rate(i)*S*MASS;
   elseif L_CE(i)>OFL
   h_SL_rate_scaled(i,1)= h_SL_rate(i).*fl_act_multiplier(i)*S*MASS;
   end
end
%% Work rate
F_CE = F_max.*activation.*fl_act_multiplier.*f_v_multiplier;
w_rate = -muscle_fiber_velocity.*F_CE;
%%  Energy rate
h_rate_scaled= h_AM_rate_scaled+h_SL_rate_scaled;

for i=1:data_length
if h_rate_scaled(i)/MASS <=1 % drop below 1 W/kg
    h_AM_rate_scaled(i,1)=1*MASS-h_SL_rate_scaled(i); 
end
end

h_rate_scaled= h_AM_rate_scaled+h_SL_rate_scaled;
E_rate       = h_rate_scaled+w_rate;

h_AM_rate=h_AM_rate_scaled;
h_SL_rate=h_SL_rate_scaled;
h_rate   =h_rate_scaled;
E_value= cumtrapz(time,E_rate);
h_rate_AM=h_AM_rate;
h_rate_SL=h_SL_rate;

MC_parameter(1,:)= E_rate;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_AM_rate;
MC_parameter(7,:)= h_SL_rate;
MC_parameter(8,:)= E_value;
end