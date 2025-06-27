function[MC_parameter,E_rate_SCALED,w_rate,h_rate_SCALED]= MC_LW07(muscle_parameter,muscle_DynCon,time,option)
% read inputs
f_FT= muscle_parameter.FT;
OFL = muscle_parameter.OFL;
MCV = muscle_parameter.MCV;
MIF = muscle_parameter.MIF;                 
PCSA= muscle_parameter.PCSA;
MASS= muscle_parameter.mass;

excitation= muscle_DynCon.muscle_excitation;
activation= muscle_DynCon.muscle_activation;
MTUforce  = muscle_DynCon.muscle_MTUforce;     

lMtilde  = muscle_DynCon.lMtilde;
vMtilde  = muscle_DynCon.vMtilde;
fl_act_multiplier=muscle_DynCon.fl_act_multiplier;
f_v_multiplier   =muscle_DynCon.f_v_multiplier;
fl_pas_multiplier=muscle_DynCon.fl_pas_multiplier;

data_length= length(time);
%% MAINTENANCE HEAT RATE
muscle_fiber_velocity    = vMtilde.*MCV*OFL;
muscle_fiber_velocity_inv= -muscle_fiber_velocity;

P=f_v_multiplier;

% Constants
G = 4; % curvature of the force-velocity curve
gamma=1.5; % According to LW05 SI

h_M_rate=zeros(data_length,1);
for i=1:data_length
    if muscle_fiber_velocity_inv(i) <0 % lengthening
        h_M_rate(i,1)=0.3*gamma*MCV./(G.^2)+0.7*gamma*MCV./(G.^2)*exp(-7*MCV*(P(i)-1)); % lo/s
    elseif muscle_fiber_velocity_inv(i) >=0 % shortening
        h_M_rate(i,1)=gamma*MCV./(G.^2); % lo/s
    end
end

X      = activation.*fl_act_multiplier;
norVal = MIF*OFL;

h_M_rate_scaled_heat= (0.3.*activation.*h_M_rate+0.7*X.*h_M_rate).*norVal;
%% SHORTENING/LENGTHENING HEAT RATE
h_SL_rate =zeros(data_length,1);
for i=1:data_length
    if muscle_fiber_velocity_inv(i) <0 % lengthening
        h_SL_rate(i,1)=-0.5*P(i).*(-vMtilde(i)).*MCV; % lo/s
    elseif muscle_fiber_velocity_inv(i) >=0 % shortening
        h_SL_rate(i,1)=((-vMtilde(i)).*MCV)./G; % lo/s
    end
end

h_SL_rate_scaled_heat= h_SL_rate.*X.*norVal; % N*m/s
%% Work rate
% Shortening velocity = +
F_CE   = MIF.*activation.*fl_act_multiplier.*f_v_multiplier;
w_rate = muscle_fiber_velocity_inv.*F_CE;
%%  Energy rate

h_rate_SCALED= h_M_rate_scaled_heat+h_SL_rate_scaled_heat;
E_rate_SCALED= w_rate+h_rate_SCALED;

E_value= cumtrapz(time,E_rate_SCALED);
h_rate_AM=h_M_rate_scaled_heat;
h_rate_SL=h_SL_rate_scaled_heat;

MC_parameter(1,:)= E_rate_SCALED;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate_SCALED;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_M_rate_scaled_heat;
MC_parameter(7,:)= h_SL_rate_scaled_heat;
MC_parameter(8,:)= E_value;
end