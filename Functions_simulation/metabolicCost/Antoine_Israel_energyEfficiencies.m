%% Example of how to compute the metabolic energy from a solution of the MRS
%---------------------------------------------------------------------------
% addpath(genpath('C:\Users\israe\Desktop\Opensim_Israel\Research\Friedl_master\MuscleRedundancySolver-master'));
addpath(genpath('C:\Users\movea\Dropbox\PhDWork\Simulation\code\MuscleRedundancySolver-master_10_2022'));
import casadi.*
% Load solution (run Walking_MRSexample.m first)
% R = load(fullfile(pwd,'Results','Walking3_Results.mat'));

modelmass = 0; %mass of the subject [kg]

Misc.idx_allMuscleList={1};
Misc.params(1,1)=3500; % Fmax
Misc.params(2,1)=0.045; % lM^0
Misc.nTrials =1;

muscle_activation=ones(1,101)*1;
lMtilde          =ones(1,101)*1;
vMtilde          =linspace(-10,0,101);
vMtildemax       =10;
            
Results.MActivation.genericMRS=muscle_activation;
Results.MExcitation.genericMRS=muscle_activation;
Results.lMtildeopt.genericMRS =lMtilde;
Results.vMtilde.genericMRS    =vMtilde;

[Fpe,FMltilde,FMvtilde] = getForceLengthVelocityProperties(lMtilde,vMtilde,vMtildemax);

Results.FMltilde.genericMRS   = FMltilde;
Results.Fpe.genericMRS        = Fpe.*Misc.params(1,1);
Results.FMvtilde.genericMRS   = FMvtilde;
Results.Time.genericMRS       = linspace(1,10,101);

DatStore.MuscleNames={'soleus_r'};

% use the post processing function to compute the metabolic energy consumption
E = GetMetabFromMRS(Results,Misc,DatStore,modelmass);
%%
% addpath(genpath('C:\Users\israe\Desktop\Opensim_Israel\Research\Friedl_master\MuscleRedundancySolver-master'));
addpath(genpath('C:\Users\movea\Dropbox\PhDWork\Simulation\code\MuscleRedundancySolver-master_10_2022'));
clc;
sub_sel = 1;
V_total_leg    = 47*opt.subjects_mass(sub_sel)*(opt.subjects_height(sub_sel)/100)+1285; % Input: Kg & meter - Output: cm^3
muscle_name    = DatStore.MuscleNames;
rho            = 1059.7;  % kg/m^3 [Umberger 2003]

muscle_parameter.MCV = vMtildemax;
muscle_parameter.MIF = Misc.params(1,1);
muscle_parameter.VOL = getVolumeFraction(muscle_name)/100*(V_total_leg./(100)^3);    % m^3 
muscle_parameter.PCSA= (muscle_parameter.VOL)./(muscle_parameter.OFL);
muscle_parameter.FT  = (1-getSlowTwitchRatios_lr(muscle_name)); % [fraction]
muscle_parameter.mass= muscle_parameter.VOL.*rho;
% muscle_parameter.mass=0.2662;

time    = Results.Time.genericMRS;
muscle_DynCon.time             = time;
muscle_DynCon.muscle_excitation= muscle_activation;
muscle_DynCon.muscle_activation= muscle_activation;

muscle_DynCon.lMtilde          = lMtilde;
muscle_DynCon.vMtilde          = vMtilde;

muscle_DynCon.fl_act_multiplier = FMltilde;
muscle_DynCon.f_v_multiplier    = FMvtilde;

muscle_DynCon.muscle_MTUforce  = [];
muscle_DynCon.fl_pas_multiplier= (Fpe)';

muscle_DynCon.F_CE = muscle_parameter.MIF.*(muscle_activation.*FMltilde.*FMvtilde);
muscle_DynCon.V_CE = vMtilde.*Misc.params(2,1);
muscle_DynCon.W_CE = -(muscle_DynCon.F_CE.*muscle_DynCon.V_CE)';

basalOn = 0;

[MC_parameter_UM03_0,~,~,~]= MC_UM03_R(muscle_parameter,muscle_DynCon,time,0,basalOn);
[MC_parameter_UM10_0,~,~,~]= MC_UM10_R(muscle_parameter,muscle_DynCon,time,0,basalOn);
[MC_parameter_UC16_0,~,~,~]= MC_UC16_R(muscle_parameter,muscle_DynCon,time,0,basalOn);
[MC_parameter_BH04_0,~,~,~]= MC_BH04_R(muscle_parameter,muscle_DynCon,time,0,basalOn);
%%
clc;
% MC_parameter=MC_parameter_UM03_0;
% title_MRS_MC = {'Umb2003'}; % Uch2016 Umb2003 Bargh2004
% figure(3);

MC_parameter=MC_parameter_UC16_0;
title_MRS_MC = {'Uch2016'}; % Uch2016 Umb2003 Bargh2004
figure(4);

clf;
subplot(2,2,1);
plot(E.genericMRS.([title_MRS_MC{:}]).Edot,'k'); hold on;
plot(MC_parameter(1,:),'r');
title({'energy rate'});

subplot(2,2,2);
plot(E.genericMRS.([title_MRS_MC{:}]).E_am,'k'); hold on;
plot(MC_parameter(4,:),'r');
plot(E.genericMRS.Umb2003.Wdot,'k','lineWidth',2); hold on;
plot(MC_parameter(2,:),'r','lineWidth',2);
title({'E_AM and work'})

subplot(2,2,3);
plot(E.genericMRS.([title_MRS_MC{:}]).E_sl,'k'); hold on;
plot(MC_parameter(5,:),'r');
title({'E_SL'})

subplot(2,2,4);
plot(MC_parameter(2,:)./MC_parameter(1,:),'r'); hold on;
plot(E.genericMRS.([title_MRS_MC{:}]).Wdot./E.genericMRS.([title_MRS_MC{:}]).Edot,'k');
title({'efficiency'})
%%
% plot with metabolic energy in the three models
figure(1);
t = Results.Time.genericMRS(1:end-1); % get time indexes on mesh points
plot(t,E.genericMRS.Bargh2004.Edot); hold on; % sum of all muscle metab powers
plot(t,E.genericMRS.Umb2003.Edot); hold on; % sum of all muscle metab powers
plot(t,E.genericMRS.Umb2010.Edot); hold on; % sum of all muscle metab powers
plot(t,E.genericMRS.Uch2016.Edot); hold on; % sum of all muscle metab powers
legend('Bargh','Umb2003','Umb2010','Uch2016');
xlabel('Time [s]');
ylabel('Metabolic power [W]');
%%
figure(2); clf;
v = Results.vMtilde.genericMRS(1:end-1);
subplot(2,2,1);
plot(v,E.genericMRS.Bargh2004.Wdot./E.genericMRS.Bargh2004.Edot); hold on; % sum of all muscle metab powers
plot( Results.vMtilde.genericMRS(1:end),MC_parameter_BH04_0(2,:)./MC_parameter_BH04_0(1,:),'r');
grid on; ylim([0 1]);

subplot(2,2,2);
plot(v,E.genericMRS.Umb2003.Wdot./E.genericMRS.Umb2003.Edot); hold on; % sum of all muscle metab powers
plot( Results.vMtilde.genericMRS(1:end),MC_parameter_UM03_0(2,:)./MC_parameter_UM03_0(1,:),'r');
grid on; ylim([0 1]);

subplot(2,2,3);
plot(v,E.genericMRS.Umb2010.Wdot./E.genericMRS.Umb2010.Edot); hold on; % sum of all muscle metab powers
plot( Results.vMtilde.genericMRS(1:end),MC_parameter_UM10_0(2,:)./MC_parameter_UM10_0(1,:),'r');
grid on; ylim([0 1]);

subplot(2,2,4);
plot(v,E.genericMRS.Uch2016.Wdot./E.genericMRS.Uch2016.Edot); hold on; % sum of all muscle metab powers
plot( Results.vMtilde.genericMRS(1:end),MC_parameter_UC16_0(2,:)./MC_parameter_UC16_0(1,:),'r');
grid on; ylim([0 1]);

xlabel('Fiber contraction [ofl/s]');
ylabel('Efficiency [ ]');