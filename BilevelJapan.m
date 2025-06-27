clc; close all; clear

currentFolder = pwd;
addpath(genpath(currentFolder));
%% Input information

% select subject and motion
SubjectSelection ='sub1';  % select subject
MotionSelection  ='v1_t1'; % read as-> v(velocity)== 1[slow], 2[normal], 3[fast]) _ t (trial)== 1st, 2nd, or 3rd trial

ModelVariation   ='';      % none
FolderOutput     ='ResultsGuna_conf1';

% read metadata
subInfo      =load(fullfile(currentFolder,'database',SubjectSelection,'model',['subject_information_' SubjectSelection '.mat']));
subject_mass =subInfo.subject_info.mass;    % [kg]
subject_height=subInfo.subject_info.height; % [cm]

gaitData =load(fullfile(currentFolder,'database',SubjectSelection,'gaitData',['gaitFeatureData_' MotionSelection '.mat']));
side_sel =gaitData.gaitData.side; % either right or left leg side

% paths and folders
ExampleFolder= ['database\' SubjectSelection];
DataFolder   = 'DataDigitalized';

Misc.OutPath        = fullfile(currentFolder,FolderOutput,SubjectSelection,MotionSelection);

% experimental data
Misc.IKfile = {fullfile(currentFolder,ExampleFolder,'IK',['IK_' SubjectSelection '_' MotionSelection '.mot'])};
Misc.IDfile = {fullfile(currentFolder,ExampleFolder,'ID',['ID_' SubjectSelection '_' MotionSelection '.sto'])};
Misc.model_path  =  fullfile(currentFolder,ExampleFolder,'model',['model_rajagopal2022_' SubjectSelection ModelVariation '.osim']);

% select the leg's side
Misc.side_sel      = side_sel;

% subject info
Misc.subject_mass   =subject_mass; % [kg]
Misc.subject_height =subject_height; % [cm]

% load parameters
variable_tuned=load(fullfile(ExampleFolder,'model',['tunedParameters_' SubjectSelection '.mat']));

% input dofs - select the DOFs you want to include in the optimization
Misc.DofNames_Input={['hip_flexion_' Misc.side_sel] ['hip_adduction_' Misc.side_sel] ['hip_rotation_' Misc.side_sel] ['knee_angle_' Misc.side_sel]  ['ankle_angle_' Misc.side_sel]}; 

% ranges
Misc.time = [0 10];
Misc.extra_frame= 5;

% label of muscle-tendon parameters
Misc.param_label=load_MTU_names();       % here I name the convention name for MTU parameters

% run muscle analysis
Misc.GetAnalysis = 1; 

% We will use previously tuned parameters
Misc.Param_read = {'selected'};
Misc.myParams   = variable_tuned.tunedParams;
  
% Setup names
muscleNames            = getMuscleNames('rajagopal');
Misc.MuscleNames_Input = appendSide(muscleNames, Misc.side_sel);

% Workflow setup
workFlow_Normal      = 1; % Normal : baseline, no assistive moment

if workFlow_Normal== 1
    Misc.OutName= 'normal';
    [Results_normal,DatStore_normal,Misc] = solveMuscleRedundancy_TV(Misc);
end
%% Check results
[J_normalized,Edot_normal_TS] = computeOuterLoopFunction(Misc,Results_normal); % this is my starting point, the Edot at unassisted conditions.
gait_cycle=linspace(0,100,length(Edot_normal_TS));
figure(1); clf; plot(gait_cycle,Edot_normal_TS); title(['net met cost = ' num2str(J_normalized,'%1.1f') 'W'])
%% Run muscle analysis for PGMs
Misc.MuscleNames_Input_PGMs={['PGMLateral_' Misc.side_sel] ['PGMMedial_' Misc.side_sel]};
Misc.DofNames_Input_PGMs   ={['hip_flexion_' Misc.side_sel] ['hip_adduction_' Misc.side_sel] ['hip_rotation_' Misc.side_sel]}; 

ModelVariation         ='_PGMsSurf_1'; % either '_PGMs' or '_PGMsSurf'
model_name_complete    =['model_rajagopal2022_' SubjectSelection ModelVariation '.osim'];
Misc.model_path        = fullfile(currentFolder,ExampleFolder,'model',model_name_complete);
MuscleAnalysisPath_PGMs= fullfile(Misc.OutPath,'MuscleAnalysis_PGM');
if ~exist(MuscleAnalysisPath_PGMs,'dir'); mkdir(MuscleAnalysisPath_PGMs); end
OpenSim_Muscle_Analysis(Misc.IKfile{1},Misc.model_path,MuscleAnalysisPath_PGMs,[Misc.time(1) Misc.time(end)],Misc.DofNames_Input_PGMs);
%% Read muscle analysis for PGMs
DatStore_PGMs=run_muscle_analysis_for_PGMs(Misc,MuscleAnalysisPath_PGMs);
%% Visualize lengths and moment arms
to_plot_PGMsGeometry=0;

if to_plot_PGMsGeometry==1
figure(2); clf; set(gcf,'color','w','Visible','on','WindowState', 'maximized'); clc; 
nPGMs=length(Misc.MuscleNames_Input_PGMs);
nDOFs=length(Misc.DofNames_Input_PGMs); 
for iPGMs=1:nPGMs
    for iDOFs=1:nDOFs
        subplot(nPGMs,nDOFs+1,iDOFs+(nDOFs+1)*(iPGMs-1))
        plot(DatStore_PGMs.dM(:,iDOFs,iPGMs));
        title([DatStore_PGMs.MuscleNames{iPGMs} '_' DatStore_PGMs.DOFNames{iDOFs}],'Interpreter','none');
        ylabel('moment arm [m]');
        xlabel('frame [#]');
    end
end

for iPGMs=1:nPGMs
    subplot(nPGMs,nDOFs+1,(nDOFs+1)*(iPGMs))
    plot(DatStore_PGMs.LMT(:,iPGMs));
    title([DatStore_PGMs.MuscleNames{iPGMs} '_length'],'Interpreter','none');
    ylabel('length [m]');
    xlabel('frame [#]');
end

sgtitle(model_name_complete,'interpreter','none') 
saveas(gcf,fullfile(Misc.OutPath,'PGMs_lengths_and_momentArms'),'png');
end
%% Run sample, self-selected parameters
to_run_example=0;

if to_run_example==1
    clf;
    % actuators info
    PGMinfo.names=DatStore_PGMs.MuscleNames;
    PGMinfo.DOFs =DatStore_PGMs.DOFNames;

    % geometry parameters
    PGMinfo.geometry.length    = DatStore_PGMs.LMT;
    PGMinfo.geometry.moment_arm= DatStore_PGMs.dM;

    % instrinsic actuator properties
    PGMinfo.intrinsic.slack_length = [0.20 0.20]; % [m]
    PGMinfo.intrinsic.time_lag     = [0.01 0.01]; % [s] (so-called tau)

    % control profile parameters
    PGMinfo.torque.stiffness = [1500 1500]; % [N/m]
    PGMinfo.torque.startTime = [60 40]; % [gait cycle %]
    PGMinfo.torque.duration  = [55 65]; % [gait cycle %]

    % define constraints
    PGMinfo.constraint.actuationLim=1; % the duration is recomputed such that the actuation timing is max 100%GC

    time           = Results_normal.Time.genericMRS;
    extra_frame    = Misc.extra_frame;
    opt_visual     = 0;                         % turn to 1 for plotting all actuator info
    [PGMinfo,~,PGMmoment,gait_cycle]  = PGMactuation_force(PGMinfo,time,extra_frame,opt_visual);
    PGMmoment_net  = squeeze(sum(PGMmoment,1)); % summed of the two actuators
    figure(3); clf;
    for i=1:length(PGMinfo.DOFs); subplot(1,length(PGMinfo.DOFs),i);
        hold on;
        plot(gait_cycle,squeeze(PGMmoment(1,i,:)),'--r','LineWidth',2); plot(gait_cycle,squeeze(PGMmoment(2,i,:)),'--b','LineWidth',2); plot(gait_cycle,PGMmoment_net(i,:),'k','LineWidth',2);
        title(PGMinfo.DOFs{i},'Interpreter','none');
    end
    Misc.torqueProfile = {PGMmoment_net(1,:) PGMmoment_net(2,:) PGMmoment_net(3,:)};

    % run muscle analysis
    Misc.GetAnalysis = 0;
    Misc.Exo_Enable  = 1; % enable assistive moment
    Misc.Exo_Mode    = "manual";                 Misc.Exo_Type="active";
    Misc.Exo_Dof     = Misc.DofNames_Input_PGMs; Misc.Exo_group=[1 1 1]; % no need to change, signs are given already
    Misc.OutName     = 'hipExo_PGM_sample';
    Misc.to_save_results=1; % it won't print values

    [Results_assisted,DatStore,Misc] = solveMuscleRedundancy_TVCUT(Misc,DatStore_normal);

    figure(4); clf;
    for i=1:40
        subplot(5,8,i)
        hold on
        plot(Results_normal.MActivation.genericMRS(i,:),'k')
        plot(Results_assisted.MActivation.genericMRS(i,:),'r')
        ylim([0 1])
        title(Results_normal.MuscleNames{i})
    end
end
%% Bilevel formulation
% Here we will use Bayesian optimization. According to my previous study,
% Bayesian converges faster than Particle Swarm optimization and Genetic
% algorithm. I chose Bayesian for this implementation
% Here are the main components
% 
% - OUTER LEVEL
% OPT=Bayesian Optimization, AIM=min(E)
% - INNER LEVEL
% OPT=Direct collocation, AIM=min(a^2)
% - PARAMETRIZATION
% PGM: stiffness, start time, duration 
%% Framework bilevel
clc;

% setup inner loop
Misc.GetAnalysis = 0;
Misc.Exo_Enable  = 1; % exo enabled
Misc.Exo_Mode    = "manual";                 Misc.Exo_Type="active";
Misc.Exo_Dof     = Misc.DofNames_Input_PGMs; Misc.Exo_group=[1 1 1]; % no need to change, signs are given already
Misc.OutName     = 'iterations';
Misc.to_save_results=0; % it wont print values

% chose configuration
conf_type={'independent'}; % <---------------------------------------------------------------------------------------------------------------------------

% set limits
vlim.k  =[500 3000];
vlim.t_s=[ 30   70];
vlim.d  =[  5   50];

if strcmp(conf_type,'coupled')
    % DEFINITION OF VARIABLES
    % Symbol  K      t_s    d
    % units   N/m    %GC    %GC
    lb_list=[vlim.k(1) vlim.t_s(1) vlim.d(1)]; % lower bound
    ub_list=[vlim.k(2) vlim.t_s(2) vlim.d(2)]; % upper bound, to be aware: MAX(t_s)+MAX(d)<=100%, because exo torque cannot exceed a full gait cycle (100%)
    varNames={'K' 't_s' 'd'}; varNames_full={'stiffness' 'startTime' 'duration'};
    varUnits={'N/m' '%GC' '%GC'};
    nVars  =length(varNames);
    nIters =100;
elseif strcmp(conf_type,'independent')
    % DEFINITION OF VARIABLES
    % Symbol  K_l   K_m  t_s_l  t_s_m   d_l   d_m
    % units   N/m   N/m    %GC    %GC   %GC   %GC
    lb_list=[vlim.k(1) vlim.k(1) vlim.t_s(1) vlim.t_s(1) vlim.d(1) vlim.d(1)]; % lower bound
    ub_list=[vlim.k(2) vlim.k(2) vlim.t_s(2) vlim.t_s(2) vlim.d(2) vlim.d(2)]; % upper bound, to be aware: MAX(t_s)+MAX(d)<=100%, because exo torque cannot exceed a full gait cycle (100%)
    varNames={'K_l' 'K_m' 't_s_l' 't_s_m' 'd_l' 'd_m'}; varNames_full={'L_stiffness' 'M_stiffness' 'L_startTime' 'M_startTime' 'L_duration' 'M_duration'};
    varUnits={'N/m' 'N/m' '%GC'   '%GC'   '%GC' '%GC'};
    nVars  =length(varNames);
    nIters =200;
else
    disp('need to choose an actuation type!')
end

% save conf
Misc.bilevelSetup.conf_type =conf_type;
Misc.bilevelSetup.varNames  =varNames;
Misc.bilevelSetup.varNames_full  =varNames_full;
Misc.bilevelSetup.varUnits  =varUnits;

var_containers = [];
for i = 1:nVars
    var_name = varNames{i}; % prior-> ['var', num2str(i)]; % Generate variable name dynamically
    var_i = optimizableVariable(var_name, [lb_list(i), ub_list(i)]); % v1 = optimizableVariable('var1',[lb(1) ub(1)]);
    var_containers = [var_containers, var_i]; % Append the new variable to the array
end

% actuators info
PGMinfo.names=DatStore_PGMs.MuscleNames;
PGMinfo.DOFs =DatStore_PGMs.DOFNames;

% geometry parameters
PGMinfo.geometry.length    = DatStore_PGMs.LMT;
PGMinfo.geometry.moment_arm= DatStore_PGMs.dM;

% instrinsic actuator properties
PGMinfo.intrinsic.slack_length = [0.20 0.20]; % [m]
PGMinfo.intrinsic.time_lag     = [0.01 0.01]; % [s] (so-called tau)

% define constraints
PGMinfo.constraint.actuationLim=1; % the duration is recomputed such that the actuation timing is max 100%GC

% FUNCTION TO BE EVALUATED, INNER OPTIMIZER IS WITHIN THIS FUNCTION
funMRS = @(x) MuscleRedundancyAndFuncAnalysis(Misc,PGMinfo,DatStore_PGMs,DatStore_normal,{x},J_normalized); 

% SETUP AND RUN THE OUTER OPTIMIZER
MaxObjectiveEvaluations=nIters;
timeExec_1=datetime('now');
resultsBayesopt = bayesopt(funMRS,var_containers,'IsObjectiveDeterministic',true,...
                           'MaxObjectiveEvaluations',MaxObjectiveEvaluations,"UseParallel",true,...
                           'AcquisitionFunctionName','expected-improvement-plus','ExplorationRatio',0.5); % ExplorationRatio is 0.5 by default
timeExec_2=datetime('now'); duration_loop=seconds(duration(timeExec_2-timeExec_1)); disp(['simulation took ' num2str(duration_loop,'%1.2f') ' secs']);
%% Summary of results & retrieve optimal
% read results comprehensively
clc;
MinObjective   =resultsBayesopt.MinObjective;
paramsAtIters  =resultsBayesopt.XTrace;
valuesAtIters  =resultsBayesopt.ObjectiveTrace;
XAtMinObjective=resultsBayesopt.XAtMinObjective;
ObjectiveMinimumTrace=resultsBayesopt.ObjectiveMinimumTrace;

bestIter  =find(valuesAtIters==MinObjective);
bestParams=XAtMinObjective{1,:};

% get PGMinfo at best, updates if required
time_series    = Misc.time(1):0.01:Misc.time(end);
extra_frame    = Misc.extra_frame;

if strcmp(conf_type,'coupled')
   param_bilevel=[bestParams(1) bestParams(1); bestParams(2) bestParams(2); bestParams(3) bestParams(3)];
elseif strcmp(conf_type,'independent')
   param_bilevel=[bestParams(1) bestParams(2); bestParams(3) bestParams(4); bestParams(5) bestParams(6)];
end
PGMinfo.torque.stiffness = [param_bilevel(1,:)]; % [N/m]
PGMinfo.torque.startTime = [param_bilevel(2,:)]; % [gait cycle %]
PGMinfo.torque.duration  = [param_bilevel(3,:)]; % [gait cycle %]

if PGMinfo.constraint.actuationLim==1
    [PGMinfo,~,~,~]  = PGMactuation_force(PGMinfo,time_series,extra_frame,0);
end

% calculate relative value
range_list=ub_list-lb_list;
bestParams_per=(bestParams-lb_list)./range_list*100;

disp(['Outer objective, metabolic cost change ' num2str(MinObjective,'%1.2f') '%'])
disp('Optimal control parameters:');
for iVar=1:nVars
    disp([varNames_full{iVar} '= ' num2str(bestParams(iVar),'%1.1f') ' ' varUnits{iVar} ', [limits,' num2str(bestParams_per(iVar),'%1.1f') '%]']);
end

% identify threashold
thres=99; minObjATthres=MinObjective*thres/100; iterForThres99=find(ObjectiveMinimumTrace<minObjATthres,1,'first');
thres=95; minObjATthres=MinObjective*thres/100; iterForThres95=find(ObjectiveMinimumTrace<minObjATthres,1,'first');
thres=90; minObjATthres=MinObjective*thres/100; iterForThres90=find(ObjectiveMinimumTrace<minObjATthres,1,'first');
% plot(valuesAtIters,'.b','MarkerSize',20); hold on; plot(ObjectiveMinimumTrace,'og'); xline(iterForThres99); xline(iterForThres95); xline(iterForThres90);

% save results, key data easy to retrieve later
bayesianResult.resultsBayesopt =resultsBayesopt;
bayesianResult.duration_loop   =duration_loop;
bayesianResult.lb_limit = lb_list; % lower bound
bayesianResult.ub_limit = ub_list; % upper bound
bayesianResult.Xbest_val= XAtMinObjective;
bayesianResult.Xbest_per= bestParams_per;
bayesianResult.iterBest.optimal=bestIter;
bayesianResult.iterBest.thres99=iterForThres99;
bayesianResult.iterBest.thres95=iterForThres95;
bayesianResult.iterBest.thres90=iterForThres90;
bayesianResult.conf_type       =conf_type;

save(fullfile(Misc.OutPath,'BayesianOpt'),'bayesianResult');
%% Run results from best iteration
[~,~,PGMmoment,gait_cycle]  = PGMactuation_force(PGMinfo,time_series,extra_frame,0); % turn to 1 for plotting all actuator info
PGMmoment_net  = squeeze(sum(PGMmoment,1)); % summed of the two actuators
Misc.torqueProfile = {PGMmoment_net(1,:) PGMmoment_net(2,:) PGMmoment_net(3,:)};

Misc.OutName    = 'bilevel';
Misc.to_save_results=1; 
[Result,~,Misc] = solveMuscleRedundancy_TVCUT(Misc,DatStore_normal);

[J_best,Edot_bilevel_TS] = computeOuterLoopFunction(Misc,Result);
%% Plot best iteration
figure(10); clf; set(gcf,'color','w','Visible','on','WindowState', 'maximized'); clc; 
nPGMs= length(PGMinfo.names);
nDOFs= size(PGMinfo.geometry.moment_arm,2);

fSel=1+extra_frame:size(Result.MActivation.genericMRS,2)-1-extra_frame;

opt_range_values=[min(PGMmoment,[],'all') max(PGMmoment,[],'all')];
range_val =opt_range_values(2)-opt_range_values(1);

subplot(2,3,1)
for iVar=1:nVars
    text(10,90-12*(iVar-1),[varNames_full{iVar} '= ' num2str(bestParams(iVar),'%1.1f') ' ' varUnits{iVar} ', [limits,' num2str(bestParams_per(iVar),'%1.1f') '%]'],'fontSize',12,'interpreter','none');
end
xlim([0 100]); ylim([0 100])
set(gca,'box','off','XTickLabel',[],'XTick',[],'YTickLabel',[],'YTick',[]);
title('optimal parameters','FontSize',15)


for iPGM=1:nPGMs
    subplot(2,3,1+iPGM);
    hold on;
    for iDOF=1:nDOFs
        plot(gait_cycle,squeeze(PGMmoment(iPGM,iDOF,:)),'LineWidth',2)
    end

    legend(PGMinfo.DOFs,'Interpreter','none','location','southwest'); legend boxoff
    set(gca,'FontSize',15);
    ylabel('moment [Nm]'); xlabel('gait cycle [%]')
    xlim([0 100])
    ylim([opt_range_values(1)-0.1*range_val opt_range_values(2)+0.1*range_val])
    title(['optimal moment for ' PGMinfo.names{iPGM}],'FontSize',15)
end

subplot(2,3,4)
plot(valuesAtIters,'.b','MarkerSize',20); hold on; plot(ObjectiveMinimumTrace,'og'); xline(iterForThres99); xline(iterForThres95); xline(iterForThres90);
text(5,0.95*max(valuesAtIters),['iOpt=' num2str(bestIter,'%1.0f') '  i99%=' num2str(iterForThres99,'%1.0f') '  i95%=' num2str(iterForThres95,'%1.0f') '  i90%=' num2str(iterForThres90,'%1.0f')],'fontSize',15)
xlabel('#Iterations'); ylabel('metabolic cost reduction [%]');
set(gca,'FontSize',15);
title('iterations','FontSize',15)

subplot(2,3,5);
plot(gait_cycle(fSel),Edot_normal_TS,'k','LineWidth',2); hold on
plot(gait_cycle(fSel),Edot_bilevel_TS,'r','LineWidth',2); hold on
set(gca,'FontSize',15);
ylabel('metabolic rates (one leg) [W/kg]'); xlabel('gait cycle [%]')
title('time series')

J_real=(mean(Edot_bilevel_TS)-mean(Edot_normal_TS))/mean(Edot_normal_TS)*100; % CHECK YOUR RESULTS (2!)

subplot(2,3,6);
x_label={'Normal' 'Bilevel'};
X = categorical(x_label);
X = reordercats(X,x_label);
data_val=[J_normalized J_best];
color_bar={'k' 'r'};

for j=1:length(x_label)
    data= data_val(j);
    hold on;
    bar(X(j),data,'FaceColor',color_bar{j})

    if j>1 % compute relative change compared to the first one
        E_normal=data_val(1);
        change_per=(data-E_normal)/E_normal*100;
        text(j,data+0.25,[num2str(change_per,'%+1.2f') ' %'],'HorizontalAlignment','center','FontSize',15)
    end
end
set(gca,'FontSize',15);
ylabel('metabolic rates (both legs) [W/kg]')
title('mean values')

sgtitle([SubjectSelection '_' MotionSelection '__conf_' conf_type{1}],'interpreter','none') 

% to save figures
saveas(gcf,fullfile(Misc.OutPath,'bilevel_summary'),'png');
pause(0.5); 

summaryInFolder=fullfile(currentFolder,FolderOutput,'summaryInOneFolder');
if ~exist(summaryInFolder,'dir'); mkdir(summaryInFolder); end
saveas(gcf,fullfile(summaryInFolder,[SubjectSelection MotionSelection]));
%%
function [J] = MuscleRedundancyAndFuncAnalysis(Misc,PGMinfo,DatStore_PGMs,DatStore_normal,exoParam,J_normalized)
% UNLOADING
% model_path= Misc.model_path;
time_series= Misc.time(1):0.01:Misc.time(end);
extra_frame= Misc.extra_frame;

actuation_type=Misc.bilevelSetup.conf_type;
varNames      =Misc.bilevelSetup.varNames;
% OutPath   = Misc.OutPath;
% trc_time  = Misc.trc_time;

% ACTUATOR DEFINITION
% actuators info
PGMinfo.names=DatStore_PGMs.MuscleNames;
PGMinfo.DOFs =DatStore_PGMs.DOFNames;

% geometry parameters
PGMinfo.geometry.length    = DatStore_PGMs.LMT;
PGMinfo.geometry.moment_arm= DatStore_PGMs.dM;

% generate a torque profile based on PGM parametrization
% control profile parameters
if strcmp(actuation_type,'coupled')
PGMinfo.torque.stiffness = [exoParam{1}.(varNames{1}) exoParam{1}.(varNames{1})]; % [N/m]
PGMinfo.torque.startTime = [exoParam{1}.(varNames{2}) exoParam{1}.(varNames{2})]; % [gait cycle %]
PGMinfo.torque.duration  = [exoParam{1}.(varNames{3}) exoParam{1}.(varNames{3})]; % [gait cycle %]
elseif strcmp(actuation_type,'independent')
PGMinfo.torque.stiffness = [exoParam{1}.(varNames{1}) exoParam{1}.(varNames{2})]; % [N/m]
PGMinfo.torque.startTime = [exoParam{1}.(varNames{3}) exoParam{1}.(varNames{4})]; % [gait cycle %]
PGMinfo.torque.duration  = [exoParam{1}.(varNames{5}) exoParam{1}.(varNames{6})]; % [gait cycle %]
end

[~,~,PGMmoment,~]  = PGMactuation_force(PGMinfo,time_series,extra_frame,0); % turn to 1 for plotting all actuator info
PGMmoment_net  = squeeze(sum(PGMmoment,1)); % summed of the two actuators
Misc.torqueProfile = {PGMmoment_net(1,:) PGMmoment_net(2,:) PGMmoment_net(3,:)};

% RUN THE INNER LOOP
try
    % [Result,~,Misc] = solveMuscleRedundancy_TV(Misc);
    [Result,~,Misc] = solveMuscleRedundancy_TVCUT(Misc,DatStore_normal);
    [J,~]=computeOuterLoopFunction(Misc,Result);
catch ME
     J=100; % if simulation fails, the optimizer does not benefit from it
end

J=(J-J_normalized)/J_normalized*100; % as a percentage of the unassisted condition
end

function [J,Edot_normal_TS]=computeOuterLoopFunction(Misc,Results)

% window of analysis
frame_extra=5;
fSel=1+frame_extra:size(Results.MActivation.genericMRS,2)-1-frame_extra;

% number of muscles
NMuscle=length(Results.MuscleNames);

% get the mass of the subject using the function GetModelMass
subject_mass  =Misc.subject_mass;       % or modelmass = getModelMass(Misc.model_path);
subject_height=Misc.subject_height;

% Order results
[musT_param,musE_param,mus_time,mus_dyn,states_field] = Results_states_params(Results,subject_mass,subject_height,frame_extra);

% select open or close
% % close
% Exoskeleton_aim='E'; 
% J_normalized   =  0;
% [J] = computeMetric(Results,musT_param,musE_param,mus_time,mus_dyn,states_field,Exoskeleton_aim,J_normalized);

% open
mode_basalOn=0; % Boolean: Select if you want to add a basal heat
mode_negWork=1; % Boolean: Select if you want negative heat to be dissipated, thus net energy rate is >=0

muscle_metRate=double.empty;
% muscle_work   =double.empty;
% muscle_heat   =double.empty;
for mus_opt=1:NMuscle
    mus_Dynamics= [mus_time(:)'; squeeze(mus_dyn(:,mus_opt,:))];
    [MC_parameter_BH04,~,~,~]= MC_BH04_R(musT_param(:,mus_opt),musE_param(:,mus_opt),mus_Dynamics,mode_negWork,mode_basalOn);
    muscle_metRate(mus_opt,:)= MC_parameter_BH04(1,:)/subject_mass; % normalized by mass
    
    % muscle_work(mus_opt,:)=MC_parameter_BH04(2,:)/subject_mass;
    % muscle_heat(mus_opt,:)=MC_parameter_BH04(3,:)/subject_mass;
end
% oneLeg_metRate=sum(muscle_metRate); % plot(oneLeg_metRate/subjectMass)

% aim_main=sum(oneLeg_metRate)/NMuscle;
Edot_normal_TS   =sum(muscle_metRate); % normalized by mass
Edot_normal_mean =mean(Edot_normal_TS)*2; % 2 legs
aim_main         =Edot_normal_mean;

%clf; for i=1:40; subplot(5,8,i); plot(muscle_metRate(i,:)); end; title('guna')
%close all; for i=1:40; subplot(5,8,i); plot(muscle_work(i,:)/subject_mass); end; title('guna')
% close all; for i=1:40; subplot(5,8,i); plot(muscle_heat(i,:)/subject_mass); end; title('guna')

NDof      =size(Results.RActivation.genericMRS,1);
helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)
aim_helper=sum(helper_obs,'all');

J=aim_main+0.1*aim_helper;


% % use the post processing function to compute the metabolic energy consumption
% Results.E= GetMetabFromMRS(Results,Misc,modelmass);
% Results.Edot.genericMRS=Results.E.genericMRS.Bargh2004.Edot;
% 
% Edot_normal      =Results.Edot.genericMRS;
% Edot_normal_TS   =sum(Edot_normal)/NMuscle;
% Edot_normal_mean =mean(Edot_normal_TS)*2; % 2 legs
% 
% % objectives and helper
% aim_main=Edot_normal_mean;
% 
% NDof      =size(Results.RActivation.genericMRS,1);
% helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)
% aim_helper=sum(helper_obs,'all');
% 
% J=aim_main+0.1*aim_helper;
end

% compute: actuator length and moment arm
function [PGMparam] = DEVactuation_geometry(PGMparam,trc_header,trc_time,trc_data_conv,markers_list,opt_visual) 

    data_length=size(trc_data_conv,1);

    % proximal marker
    ind_proximal           = find(strcmp(trc_header,PGMparam.user.proximal_marker_reference)); 
    proximal_marker_user   = trc_data_conv(:,((ind_proximal-1)*3+1)+[0 1]); % x and y
    proximal_marker_device = proximal_marker_user + PGMparam.user.proximal_marker_offset; % TO FIX - RIGID BODY ASSUMPTION NOT HERE
    
    % distal marker
    ind_distal           = find(strcmp(trc_header,PGMparam.user.distal_marker_reference)); 
    distal_marker_user   = trc_data_conv(:,((ind_distal-1)*3+1)+[0 1]); % x and y
    distal_marker_device = distal_marker_user + PGMparam.user.distal_marker_offset; % TO FIX - RIGID BODY ASSUMPTION NOT HERE
    
    ind_HJC              = find(strcmp(trc_header,PGMparam.user.HJC_marker_reference)); 
    HJC_marker_user      = trc_data_conv(:,((ind_HJC-1)*3+1)+[0 1]); % x and y

    % length
    PGM_length=zeros(data_length,1);
    for i=1:data_length
        PGM_length(i)=norm(proximal_marker_device(i,:)-distal_marker_device(i,:));
    end

    % moment arm (respect to the hip joint center)
    PGM_moment_arm = perpendicular_distances(distal_marker_device, proximal_marker_device, HJC_marker_user);

    PGMparam.geometry.length=PGM_length;
    PGMparam.geometry.moment_arm=PGM_moment_arm;

    % visualization
    if opt_visual.flag==1
        figure;

        subplot(3,1,1)
        plot(trc_time,PGM_length)
        xlim([trc_time(1) trc_time(end)])
        ylabel('PGM length [m]'); xlabel('time [s]')

        subplot(3,1,2)
        plot(trc_time,PGM_moment_arm)
        xlim([trc_time(1) trc_time(end)])
        ylabel('PGM moment arm, HJC [m]'); xlabel('time [s]')

        subplot(3,1,3)
        numMakers_list=length(markers_list);
        marker_ind=zeros(1,numMakers_list);
        for i=1:numMakers_list
            marker_ind(i)=find(strcmp(trc_header,markers_list(i)));
        end
        axis_choosen= [1 2]; % these numbers correspond to indexes aligned to X (horizontal) and Y (vertical) axes

        for i=1:10:data_length
            for j=1:numMakers_list
                marker_sel = marker_ind(j);
                ind        = ((marker_sel-1)*3+1) + (axis_choosen-1);

                if strcmp(markers_list{j}(1),'L'); color_marker='r'; else; color_marker='g'; end

                plot(trc_data_conv(i,ind(1)),trc_data_conv(i,ind(2)),'.','MarkerSize',10,'Color',color_marker);
                hold on;
            end
            plot(distal_marker_device(i,1),distal_marker_device(i,2),'.','MarkerSize',20,'Color','b');
            plot(proximal_marker_device(i,1),proximal_marker_device(i,2),'.','MarkerSize',20,'Color','b');

            axis([-2.3 2.7 -0.025 2.00])
            hold off;
            title(['marker trajectories t=' num2str(trc_time(i),'%1.2f')  's'])
            ylabel('vertical [m]'); xlabel('horizontal [m]')
            pause(0.10)
        end
    end
end

% function [PGM_force,PGM_moment]        = PGMactuation_force(PGMparam,trc_time,time,opt_visual)
%     ind_ti=find(trc_time==time(1));
%     ind_tf=find(trc_time==time(end));
% 
%     time_series = time(1):0.01:time(end);
%     data_length = length(time_series);
%     gait_cycle  = linspace(0,100,length(time_series));
% 
%     % control signal generation
%     control_signal=zeros(data_length,1); % defined
%     ind =gait_cycle>PGMparam.torque.startTime & gait_cycle<(PGMparam.torque.startTime+PGMparam.torque.duration); % active time
%     control_signal(ind)=1;
% 
%     % control signal - smoothed, 1st order adopted to mimic slow increase and
%     % decrease of the pressure that fills the gel muscle actuator
% 
%     % Parameters
%     tau = PGMparam.actuator.time_lag; %(s) It must be changed according to actuator properties (EXP required)
%     y0  = 0;    % Initial condition
% 
%     % Time span for the solution
%     tspan = time_series; % From t = 0 to t = 1
%     u     = control_signal;
% 
%     u_interp = @(t) interp1(time_series, u, t, 'linear', 'extrap'); % Interpolating function for u(t)
%     ode = @(t, y) (u_interp(t) - y) / tau; % first order differential eq.
%     [t_int, y_int] = ode45(ode, tspan, y0); % Solve the ODE
% 
%     control_signal_ode=y_int;
% 
%     % compute forces: spring and damping
% 
%     exo_delta            = PGMparam.geometry.length(ind_ti:ind_tf)-PGMparam.actuator.slack_length;
%     exo_active_delta_len = exo_delta.*control_signal;            % without ode smoothing (not used)
%     exo_active_delta_len_smooth=exo_delta.*control_signal_ode; % with ode smoothing
% 
%     % exo_active_delta_vel=gradient(exo_active_delta_len_smooth)./gradient(time'); % velocity
%     spring_force   =PGMparam.torque.stiffness*exo_active_delta_len_smooth; % spring force
%     % damping_force  =-abs(exo_active_delta_vel*exo_damping); % damping force - compute
% 
%     actuator_force =spring_force; %+damping_force; % total force
% 
%     actuator_force_only_pos=zeros(data_length,1);
%     actuator_force_only_pos(actuator_force>0)=actuator_force(actuator_force>0);
% 
%     PGM_force=actuator_force_only_pos;
% 
%     PGM_moment=PGMparam.geometry.moment_arm(ind_ti:ind_tf).*PGM_force;
% 
%     if opt_visual.flag ==1
%         figure;
% 
%         % length
%         PGMparam_length=PGMparam.geometry.length(ind_ti:ind_tf);
%         PGMparam_length_min=min(PGMparam_length);
%         PGMparam_length_max=max(PGMparam_length);
%         PGMparam_length_tot=PGMparam_length_max - PGMparam_length_min;
% 
%         subplot(2,2,1)
%         ind_actuator=find(ind);
%         hold on;
%         plot(time_series,PGMparam_length)
%         yline(PGMparam.actuator.slack_length,'--r')
%         xline(time_series(ind_actuator(1)),':k')
%         xline(time_series(ind_actuator(end)),':k')
%         % axis([time_series(1) time_series(end) PGMparam_length_min-0.1*PGMparam_length_tot PGMparam_length_max+0.1*PGMparam_length_tot])
%         xlim([time_series(1) time_series(end)])
%         xlabel('time [s]'); ylabel('length [m]');
% 
%         % moment arm
%         PGMparam_length=PGMparam.geometry.moment_arm(ind_ti:ind_tf);
%         PGMparam_length_min=min(PGMparam_length);
%         PGMparam_length_max=max(PGMparam_length);
%         PGMparam_length_tot=PGMparam_length_max - PGMparam_length_min;
% 
%         subplot(2,2,2)
%         plot(time_series,PGMparam_length)
%         xline(time_series(ind_actuator(1)),':k')
%         xline(time_series(ind_actuator(end)),':k')
%         axis([time_series(1) time_series(end) PGMparam_length_min-0.1*PGMparam_length_tot PGMparam_length_max+0.1*PGMparam_length_tot])
%         xlabel('time [s]'); ylabel('moment arm [m]');
% 
%         subplot(2,2,3)
%         plot(time_series,PGM_force)
%         axis([time_series(1) time_series(end) 0 1.1*max(PGM_force)])
%         xlabel('time [s]'); ylabel('force [N]');
% 
%         subplot(2,2,4)
%         plot(time_series,PGM_moment)
%         axis([time_series(1) time_series(end) 0 100])
%         xlabel('time [s]'); ylabel('moment [Nm]');
%     end
% end

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

function newNames = appendSide(muscleNames, side)
    % appendSide appends 'r' or 'l' to each muscle name based on input side
    % Input:
    %   muscleNames - cell array of strings ending in '_'
    %   side        - character or string, 'r' or 'l'
    % Output:
    %   newNames    - cell array of strings with 'r' or 'l' appended

    % Validate input
    if ~ismember(side, {'r', 'l'})
        error('Side must be either ''r'' or ''l''.');
    end

    % Append side to each name
    newNames = cellfun(@(name) [name side], muscleNames, 'UniformOutput', false);
end

function muscleNames = getMuscleNames(model_name)
    if strcmp(model_name,'rajagopal')
       muscleNames= {'addbrev_' 'addlong_' 'addmagDist_' 'addmagIsch_' 'addmagMid_' 'addmagProx_' 'bflh_' ...
                          'bfsh_'    'edl_'     'ehl_'        'fdl_'        'fhl_'       'gaslat_'     'gasmed_' 'glmax1_' ...
                          'glmax2_'  'glmax3_'  'glmed1_'     'glmed2_'     'glmed3_'    'glmin1_'     'glmin2_' 'glmin3_' ...
                          'grac_'    'iliacus_' 'perbrev_'    'perlong_'    'piri_'      'psoas_'      'recfem_' 'sart_' ...
                          'semimem_' 'semiten_' 'soleus_'     'tfl_'        'tibant_'    'tibpost_'    'vasint_' 'vaslat_' 'vasmed_'};
    else
        error('Provide a valid model');
    end
end
function [DatStore_PGMs]=run_muscle_analysis_for_PGMs(Misc,MuscleAnalysisPath_PGMs)
DOF_inds=nan(length(Misc.DofNames_Input_PGMs),1);
ct=1;

% Loop over each DOF in the model
for i=1:length(Misc.DofNames_Input_PGMs)
    
    % read the Muscle Analysis Result
    MA_FileName=fullfile(MuscleAnalysisPath_PGMs,[Misc.trialName '_MuscleAnalysis_MomentArm_' Misc.DofNames_Input_PGMs{i} '.sto']);
    if exist(MA_FileName,'file')
        dm_Data_temp=importdata(fullfile(MuscleAnalysisPath_PGMs,[Misc.trialName '_MuscleAnalysis_MomentArm_' Misc.DofNames_Input_PGMs{i} '.sto']));
    else
        error(['Cannot open muscle analysis results for: ' Misc.DofNames_Input_PGMs{i}])
    end
    
    % get the indexes for the selected MuscleNames (only needed in first iteration)
    if i==1
        nfr = length(dm_Data_temp.data(:,1));
        headers=dm_Data_temp.colheaders;
        Inds_muscles=nan(length(Misc.MuscleNames_Input_PGMs),1);
        ctm=1;
        for j=1:length(Misc.MuscleNames_Input_PGMs)
            ind_sel=find(strcmp(Misc.MuscleNames_Input_PGMs{j},headers));
            if ~isempty(ind_sel)
                Inds_muscles(ctm)=ind_sel; IndsNames_sel(ctm)=j;
                ctm=ctm+1;
            else
                disp(['Warning: The selected muscle ' Misc.MuscleNames_Input_PGMs{j} ' does not exist in the selected model. This muscles is removed from the program']);
            end
        end
        Misc.MuscleNames_PGMs=Misc.MuscleNames_Input_PGMs(IndsNames_sel);
        Inds_muscles(isnan(Inds_muscles))=[];                               % Delete the muscles names that are not selected by the user
        dM_temp=nan(nfr,length(Misc.DofNames_Input_PGMs),length(Misc.MuscleNames_PGMs));    % pre-allocate moment arms
    end
    
    % Evaluate if one of the muscles spans this DOF (when moment arms > 0.001)
    dM=dm_Data_temp.data(:,Inds_muscles);    
    if any(any(abs(dM)>0.001))
        Misc.DofNames_muscles{ct}=Misc.DofNames_Input_PGMs{i};
        dM_temp(:,i,:)=dM;
        DOF_inds(ct)=i;
        ct=ct+1;   
    end     
end

% Combine DOFs_actuated by muscles and the DOFS selected by the user
ct=1;
Inds_deleteDOFS=zeros(length(Misc.DofNames_muscles),1);
for i=1:length(Misc.DofNames_muscles)
    if ~any(strcmp(Misc.DofNames_Input_PGMs,Misc.DofNames_muscles{i}))
         Inds_deleteDOFS(ct)=i;ct=ct+1;
    end
end
Inds_deleteDOFS(ct:end)=[];
DOF_inds(Inds_deleteDOFS)=[];
Misc.DofNames=Misc.DofNames_muscles; Misc.DofNames(Inds_deleteDOFS)=[];

% warnings when not all the input DOFS are actuated by muscles
for i=1:length(Misc.DofNames_Input_PGMs)
    if ~any(strcmp(Misc.DofNames_Input_PGMs{i},Misc.DofNames))
        disp(['Warning DOF: The input dof: ' Misc.DofNames_Input_PGMs{i} ' is not actuated by the selected muscles and therefore removed from the analysis']);
    end
end

% Filter the moment arms information and store them in DatStore.dM
dM_raw=dM_temp(:,DOF_inds,:);
t_dM = dm_Data_temp.data(:,1);
fs=1/mean(diff(t_dM));
[B,A] = butter(Misc.f_order_dM, Misc.f_cutoff_dM/(fs/2));
DatStore_PGMs.dM = filtfilt(B,A,dM_raw);

% filter Muscle-tendon lengths and store them in DatStore.LMT
LMT_dat=importdata(fullfile(MuscleAnalysisPath_PGMs,[Misc.trialName '_MuscleAnalysis_Length.sto']));
LMT_raw=LMT_dat.data(:,Inds_muscles);
t_lMT = LMT_dat.data(:,1);
fs=1/mean(diff(t_lMT));             % sampling frequency
[B,A] = butter(Misc.f_order_lMT,Misc.f_cutoff_lMT/(fs/2));
DatStore_PGMs.LMT = filtfilt(B,A,LMT_raw);

% store information in the DatStore structure
DatStore_PGMs.MuscleNames = Misc.MuscleNames_PGMs;
DatStore_PGMs.DOFNames    = Misc.DofNames;
DatStore_PGMs.nMuscles    = length(Misc.MuscleNames_PGMs);
DatStore_PGMs.nDOF        = length(Misc.DofNames);
end

function [moment_complete, gait_cycle] = interpolate2GaitCyle(Misc_time,Misc_toeOff_time,extra_frame,moment_spline,to_plot)

% get time as in one frame equals 0.01 s
time        =Misc_time;
time_series =time(1):0.01:time(end);
data_length =length(time_series);

% conversion to gait cycle
data_GC      = data_length-2*extra_frame;
frames_per_GC= 100/(data_GC-1);
gait_cycle   = 0-frames_per_GC*extra_frame:frames_per_GC:100+frames_per_GC*extra_frame; % case without extra frame-> gait_cycle =linspace(0,100,length(time_series));

% toe off as a percentage of the gait cycle
toeOff_time=Misc_toeOff_time;
toeOff_asGC=(toeOff_time-time(1))/(time(end)-time(1))*100;
ind_toe    =find(gait_cycle>toeOff_asGC,1,'first')-1;

% interporlation
x_original = linspace(0, toeOff_asGC, length(moment_spline));
x_new      = linspace(0, toeOff_asGC, ind_toe-extra_frame);
v_inter = interp1(x_original,moment_spline,x_new,'cubic');
% clf; hold on; plot(x_new,v_inter,'r'); plot(x_original,moment_spline,'ok');

moment_complete=zeros(length(time_series),1);
moment_complete(1+extra_frame:ind_toe)=v_inter;

if to_plot
    figure;
    stance_phase=linspace(0,100,length(moment_spline));
    subplot(1,2,1); plot(stance_phase,moment_spline); xlim([0 100]);                         xlabel('stance phase [%]'); ylabel('moment [Nm]'); title('moment vs. stance phase')
    subplot(1,2,2); plot(gait_cycle,moment_complete); xlim([gait_cycle(1) gait_cycle(end)]); xlabel('gait cycle [%]');   ylabel('moment [Nm]'); title(['moment vs. gait cycle using ' num2str(extra_frame) ' extra frames'])
    xline(0,':k');  xline(gait_cycle(ind_toe),'--r'); xline(100,':k');
end
end