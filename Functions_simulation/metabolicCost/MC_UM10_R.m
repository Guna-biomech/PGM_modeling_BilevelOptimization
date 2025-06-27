function[MC_parameter,E_rate,w_rate,h_rate]= MC_UM10_R(musT_param,musE_param,mus_Dynamics,option,basalOn)
% read inputs
FMo=musT_param(1); lMo=musT_param(2); vMmax=musT_param(5);
Mvol=musE_param(1); Mpcsa=musE_param(2); Mft=musE_param(3); Mmass=musE_param(4);
  
time       =mus_Dynamics(1,:);     FMvtilde  =mus_Dynamics(7,:);
MExcitation=mus_Dynamics(2,:);     TForce    =mus_Dynamics(8,:);
MActivation=mus_Dynamics(3,:);     FMlpas    =mus_Dynamics(9,:);
lMtildeopt =mus_Dynamics(4,:);     Fce       =mus_Dynamics(10,:);
vMtilde    =mus_Dynamics(5,:);     Vce       =mus_Dynamics(11,:); %-shortening +lengthening
FMltilde   =mus_Dynamics(6,:);     Wdotce    =mus_Dynamics(12,:); % W_CE=-F_CE*V_CE: +W_CE spent -W_CE gain

data_length= length(time);
%% PHYSIOLOGICAL PARAMETERS
% rho= 1059.7;    sigma=0.25*10^6;
S= 1.5; %Aerobic
f_ST  = 1-Mft;

VCEtilde= Vce./lMo; % [OFL/s]
L_CE= lMtildeopt*lMo;   % [m]
%% SCALING FACTOR
A=zeros(data_length,1);
for i=1:data_length
    if MExcitation(i) > MActivation(i)
    A(i,1)= MExcitation(i);
    elseif MExcitation(i) <= MActivation(i)
    A(i,1)= (MExcitation(i)+MActivation(i))/2;      
    end
end
A_AM= A.^(0.6);
A_S = zeros(data_length,1);
for i=1:data_length
    if VCEtilde(i) <= 0 %shortening
    A_S(i,1)= A(i).^2;
    elseif VCEtilde(i) > 0 %lengthening
    A_S(i,1)= A(i);
    end
end 
%% VELOCITY PARAMETERS
VCE_MAX= vMmax;

VCE_MAX_ST= VCE_MAX/2.5; %4
VCE_MAX_FT= VCE_MAX; %10

alpha_S_ST= 100/VCE_MAX_ST; % become 25
alpha_S_FT= 153/VCE_MAX_FT;
alpha_L= 0.3*alpha_S_ST;
%% ACTIVATION/MAINTENANCE HEAT RATE
h_AM_rate_scaled=zeros(data_length,1);
h_AM_rate= 128*Mft + 25; %isometric

for i=1:data_length
   if L_CE(i)<=lMo
   h_AM_rate_scaled(i,1)= h_AM_rate.*A_AM(i).*S*Mmass;
   elseif L_CE(i)>lMo
   h_AM_rate_scaled(i,1)= (0.4*h_AM_rate+0.6*h_AM_rate*FMltilde(i)).*A_AM(i).*S*Mmass;
   end
end
%% SHORTENING/LENGHTENING HEAT RATE
h_SL_rate_scaled=zeros(data_length,1);
term_conditioned=zeros(data_length,1);
h_SL_rate=zeros(data_length,1);

for i=1:data_length
   if VCEtilde(i)<=0 % shortening
      
       term_conditioned(i,1)=-(alpha_S_ST.*VCEtilde(i).*f_ST);
       
%        % Umberger constraint
% not implemented to be comparable with Antonise
%        if term_conditioned(i,1) >= 100 %alpha_S_ST*VCE_MAX_ST % constrained/limited
%           term_conditioned(i,1) = alpha_S_ST.*VCE_MAX_ST;
%        end
       
      h_SL_rate(i,1)= (term_conditioned(i) -alpha_S_FT.*VCEtilde(i).*Mft).*A_S(i);
   elseif VCEtilde(i)>0 % lengthening
      h_SL_rate(i,1)= alpha_L*VCEtilde(i).*A(i);
   end
end

for i=1:data_length
    if L_CE(i)<=lMo
        h_SL_rate_scaled(i,1)= h_SL_rate(i)                      *S*Mmass;
    elseif L_CE(i)>lMo
        h_SL_rate_scaled(i,1)= h_SL_rate(i).*FMltilde(i)*S*Mmass;
    end
end
%% WORK RATE
F_CE = FMo.*MActivation.*FMltilde.*FMvtilde;
w_rate =zeros(data_length,1); 
for i=1:data_length
   if Vce(i)<=0 % shortening
      w_rate(i,1) =-Vce(i).*F_CE(i);
   elseif Vce(i)>0 % lengthening
      w_rate(i,1) =  0;
   end
end
%% ENERGY RATE
h_rate_scaled= h_AM_rate_scaled+h_SL_rate_scaled;
E_rate       = h_rate_scaled+w_rate;

if basalOn==1
    h_rate_scaled=h_rate_scaled+1.2*Mmass; %1.2 W/kg
    E_rate=h_rate_scaled + w_rate;
end

h_AM_rate=h_AM_rate_scaled;
h_SL_rate=h_SL_rate_scaled;
h_rate=h_rate_scaled;


if data_length>1
    totalE = cumtrapz(time,E_rate);
else
    totalE=nan;
end

h_rate_AM=h_AM_rate;
h_rate_SL=h_SL_rate;

MC_parameter(1,:)= E_rate;
MC_parameter(2,:)= w_rate;
MC_parameter(3,:)= h_rate;
MC_parameter(4,:)= h_rate_AM;
MC_parameter(5,:)= h_rate_SL;

MC_parameter(6,:)= h_AM_rate;
MC_parameter(7,:)= h_SL_rate;
MC_parameter(8,:)= totalE;
end