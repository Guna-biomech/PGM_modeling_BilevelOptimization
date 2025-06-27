function[MC_parameter,E_rate_SCALED,w_rate,h_rate_SCALED]= MC_LW05(muscle_parameter,muscle_DynCon,time,option)
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
%% STABLE HEAT RATE

muscle_fiber_velocity = vMtilde.*MCV*OFL;
muscle_fiber_velocity_inv= -muscle_fiber_velocity;
P= MIF.*activation.*fl_act_multiplier.*f_v_multiplier;

% Constants
G = 4;
% Assignations
Vmax= MCV;
gamma=1.5;

% Equations
ACT= activation;
h_M_rate= ACT.*Vmax./(G^2);
norVal  = MIF*OFL;
h_M_rate_SCALED= h_M_rate*norVal*gamma; % TO ACCOUNT FOR LABILE HEAT RATE AS LW 2007

% %% Labile heat rate
% h_L_rate= h_M_rate(:).*(0.8.*exp(-0.72.*t_stim(:))+0.175.*exp(-0.022.*t_stim(:)));
h_L_rate_SCALED= zeros(data_length,1);
%% S/L HEAT RATE
h_SL_rate= ((-vMtilde).*MCV)./G; % lo/s
h_SL_rate_SCALED= h_SL_rate*norVal.*activation;
%% THERMOELASTIC HEAT RATE
muscle_force_smooth = smooth(time,P,0.05,'loess');
muscle_force_rate = gradient(muscle_force_smooth)./gradient(time);
muscle_force_rate(isnan(muscle_force_rate))=0;
h_T_rate= -0.014*muscle_force_rate;
h_T_rate_SCALED= h_T_rate*OFL;
%% Work rate
% Shortening velocity = +
w_rate = muscle_fiber_velocity_inv.*MIF.*activation.*fl_act_multiplier.*f_v_multiplier;
%%  Energy rate
for i=1:data_length
    if muscle_fiber_velocity_inv(i)>=0
         h_rate_SCALED(i,1)= h_M_rate_SCALED(i) + h_L_rate_SCALED(i) + h_SL_rate_SCALED(i) + h_T_rate_SCALED(i);
         E_rate_SCALED(i,1)= w_rate(i) + h_rate_SCALED(i,1);
    
    elseif muscle_fiber_velocity_inv(i)<=0
         h_rate_SCALED(i,1)= h_M_rate_SCALED(i).*(0.3+0.7.*exp(-8*((P(i)./ACT(i))-1) ))-w_rate(i);
         E_rate_SCALED(i,1)= w_rate(i) + h_rate_SCALED(i);
        
%     h_rate_SCALED(i)= h_M_rate_SCALED(i).*(0.3+0.7.*exp(-8*((P(i)./ACT(i))-1) ))-w_rate(i);
    % I assigned the scaling factor to the mantainance term (replacing the previous equation). 
    % All the others are zero. I do this only because I have to assign this to a
    % particular heat. It is not written in the original formula but it
    % makes sense
    
        h_M_rate_SCALED(i,1) = h_M_rate_SCALED(i).*(0.3+0.7.*exp(-8*((P(i)./ACT(i))-1) ))-w_rate(i);
        h_SL_rate_SCALED(i,1)=0;
        h_T_rate_SCALED(i,1)=0;
        h_L_rate_SCALED(i,1)=0;
    end
end

E_value= cumtrapz(time,E_rate_SCALED);
h_rate_AM=h_M_rate_SCALED + h_T_rate_SCALED + h_L_rate_SCALED;
h_rate_SL=h_SL_rate_SCALED;

MC_parameter(1,:)= E_rate_SCALED;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate_SCALED;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_M_rate_SCALED;
MC_parameter(7,:)= h_SL_rate_SCALED;
MC_parameter(8,:)= h_T_rate_SCALED;
MC_parameter(9,:)= E_value;
end