% close all; clear

currentFolder = pwd;
addpath(genpath(currentFolder));
savingFolder  = fullfile(currentFolder,'complementary'); % all new data will be saved here
%% Change this accordingly
path_results='C:\Users\gunar\Documents\MATLAB\Guna';

conf_label='Conf1';        % options: Conf1 Conf2 Conf3
type_label='coupled';      % options: coupled independent
set_label ='1';            % options: 1 2 3
SubjectSelection ='sub1';  % select subject
MotionSelection  ='v1_t1'; % read as-> v(velocity)== 1[slow], 2[normal], 3[fast]) _ t (trial)== 1st, 2nd, or 3rd trial

modelVariant_path=fullfile(currentFolder,'database',SubjectSelection,'model'); % model is here
%% Read optimal parameters
path_gen=fullfile(path_results,[conf_label '_' type_label '_' set_label],...
    SubjectSelection,MotionSelection);
% path_gen=fullfile(path_results,[conf_label '_' type_label '_' set_label],[conf_label '_' type_label '_' set_label],...
%     SubjectSelection,MotionSelection);
path_dir_bil =fullfile(path_gen, 'bilevelResults.mat');
path_dir_opti=fullfile(path_gen, 'BayesianOpt.mat');

data_R    =load(path_dir_bil);
data_opti =load(path_dir_opti);

nFrames        = 101;
%% Interpolate
% extrapolates to 101 points AND crops the extra frames
[R_int]   = interpolatedMRS(data_R,nFrames);
%% Compute met cost: time series, net cost, individual muscle cost, and muscle group cost

Results=data_R.Results;
Misc   =data_R.Misc;
[allMCM,muscles_Edot_TS,leg_Edot_TS,net_Edot_avg] = computeMetabolicCost(R_int,Misc);
mGroup                                            = computeMuscleGroupMetCost(R_int,muscles_Edot_TS);

MCM_int.allMCM         =allMCM;
MCM_int.muscles_Edot_TS=muscles_Edot_TS;
MCM_int.leg_Edot_TS    =leg_Edot_TS;
MCM_int.net_Edot_avg   =net_Edot_avg;
MCM_int.mGroup         =mGroup;

% [sum(mGroup_muscles_Edot_avg,'all')  mean(leg_Edot_TS)] % verification

to_plot_muscleGroup_Edot=0;

if to_plot_muscleGroup_Edot==1 % here an example of the muscle groups
MA_label      =R_int.MA_label;
gait_cycle=0:100;

figure(1);
for i=1:10
    subplot(2,5,i)
    plot(gait_cycle,mGroup.mGroup_Edot_TS(i,:));
    title(MA_label{i})
    ylim([0 3.5])
end
end
%% Run, save, and read PGMs geometry
%% -> note, I have to do update "Misc.IKfile", you do not need this line.
%Misc.IKfile={fullfile(currentFolder,'database',SubjectSelection,'IK',['IK_' SubjectSelection '_' MotionSelection '.mot'])};
%%
% Run muscle analysis for PGMs
Misc.MuscleNames_Input_PGMs={['PGMLateral_'  Misc.side_sel] ['PGMMedial_'     Misc.side_sel]};
Misc.DofNames_Input_PGMs   ={['hip_flexion_' Misc.side_sel] ['hip_adduction_' Misc.side_sel] ['hip_rotation_' Misc.side_sel]}; 

ModelVariation         = 'PGMsSurf'; % either '_PGMs' or '_PGMsSurf'
model_name_complete    = ['model_rajagopal2022_' SubjectSelection '_' ModelVariation '_' conf_label '.osim'];
Misc.model_path_PGMs   = fullfile(modelVariant_path,model_name_complete);
MuscleAnalysisPath_PGMs= fullfile(savingFolder,SubjectSelection,'MuscleAnalysis_PGM',conf_label,MotionSelection);
if ~exist(MuscleAnalysisPath_PGMs,'dir'); mkdir(MuscleAnalysisPath_PGMs); end
OpenSim_Muscle_Analysis(Misc.IKfile{1},Misc.model_path_PGMs,MuscleAnalysisPath_PGMs,[Misc.time(1) Misc.time(end)],Misc.DofNames_Input_PGMs);
% Read muscle analysis for PGMs
DatStore_PGMs=read_muscle_analysis_for_PGMs(Misc,MuscleAnalysisPath_PGMs);
%% Setup and save PGMinfo
%% read from optimal
XAtMinObjective=data_opti.bayesianResult.resultsBayesopt.XAtMinObjective;
bestParams=XAtMinObjective{1,:};

if strcmp(type_label,'coupled')
   param_bilevel=[bestParams(1) bestParams(1); bestParams(2) bestParams(2); bestParams(3) bestParams(3)];
elseif strcmp(type_label,'independent')
   param_bilevel=[bestParams(1) bestParams(2); bestParams(3) bestParams(4); bestParams(5) bestParams(6)];
end

%%
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
PGMinfo.torque.stiffness = [param_bilevel(1,:)]; % [N/m]
PGMinfo.torque.startTime = [param_bilevel(2,:)]; % [gait cycle %]
PGMinfo.torque.duration  = [param_bilevel(3,:)]; % [gait cycle %]

% define constraints
PGMinfo.constraint.actuationLim=1; % the duration is recomputed such that the actuation timing is max 100%GC

time           = data_R.Results.Time.genericMRS; % not interpolated (yet)
extra_frame    = Misc.extra_frame;
opt_visual     = 0;                         % turn to 1 for plotting all actuator info
[PGMinfo,PGMforce,PGMmoment,gait_cycle]  = PGMactuation_force(PGMinfo,time,extra_frame,opt_visual);

PGMdata.PGMinfo   =PGMinfo;
PGMdata.PGMforce  =PGMforce;
PGMdata.PGMmoment =PGMmoment;

%% Interpolate PGMdata
% extrapolates to 101 points AND crops the extra frames
[PGMdata_int]   = interpolatedPGMdata(PGMdata,nFrames,extra_frame);
%% Save variables

to_save_data=1;

if to_save_data==1
    folder_MRS=fullfile(savingFolder,SubjectSelection,'MRS_int');
    folder_MCM=fullfile(savingFolder,SubjectSelection,'MCM_int');
    folder_PGM=fullfile(savingFolder,SubjectSelection,'PGM_int');

    if ~exist(folder_MRS,'dir'); mkdir(folder_MRS); end
    if ~exist(folder_MCM,'dir'); mkdir(folder_MCM); end
    if ~exist(folder_PGM,'dir'); mkdir(folder_PGM); end

    save(fullfile(savingFolder,SubjectSelection,'MRS_int',[MotionSelection '_' conf_label '_' type_label '.mat']),'R_int');
    save(fullfile(savingFolder,SubjectSelection,'MCM_int',[MotionSelection '_' conf_label '_' type_label '.mat']),'MCM_int'); % this is interpolated already
    save(fullfile(savingFolder,SubjectSelection,'PGM_int',[MotionSelection '_' conf_label '_' type_label '.mat']),'PGMdata_int');
end

function [mGroup] =computeMuscleGroupMetCost(R_int,muscles_Edot_TS)
MA      =R_int.MA;
nFrames =R_int.nFrames;
nMuscles=size(R_int.MA,1);
nMGROUPS=size(MA,2);
mGroup_muscles_Edot_TS=zeros(nMuscles,nMGROUPS,nFrames);
mGroup_muscles_Edot_avg=zeros(nMuscles,nMGROUPS);
mGroup_Edot_TS=zeros(nMGROUPS,nFrames);
for iMus=1:nMuscles
    MA_total(1,:)=sum(MA(iMus,:,1:nFrames));
    for iMGROUP=1:nMGROUPS
        MAG(1,:)=MA(iMus,iMGROUP,1:nFrames);
        MA_ratio(:)=MAG(1,:)./MA_total(1,:);
        mGroup_muscles_Edot_TS(iMus,iMGROUP,:)=muscles_Edot_TS(iMus,:).*MA_ratio(:)';

        mGroup_muscles_Edot_avg(iMus,iMGROUP)=mean(mGroup_muscles_Edot_TS(iMus,iMGROUP,:));
    end
end
for iMGROUP=1:nMGROUPS
    mGroup_Edot_TS(iMGROUP,1:nFrames)=sum(mGroup_muscles_Edot_TS(:,iMGROUP,:));
end

mGroup.muscles_Edot_TS =mGroup_muscles_Edot_TS;
mGroup.muscles_Edot_avg=mGroup_muscles_Edot_avg;
mGroup.Edot_TS         =mGroup_Edot_TS;
end

function [PGMdata_int] = interpolatedPGMdata(PGMdata,nFrames,extra_frame)
data_length=length(PGMdata.PGMinfo.geometry.length);
iSel=1+extra_frame:data_length-extra_frame;
length_int=interpolation_alone(PGMdata.PGMinfo.geometry.length(iSel),nFrames);

nActs=length(PGMdata.PGMinfo.names);
nDOFs=length(PGMdata.PGMinfo.DOFs);

moment_arm_int=zeros(nFrames,nDOFs,nActs);
for iAct=1:nActs
    for iDOF=1:nDOFs
        moment_arm_int(:,iDOF,iAct)=interpolation_alone(PGMdata.PGMinfo.geometry.moment_arm(iSel,nDOFs,iAct),nFrames);
    end
end

PGMinfo=PGMdata.PGMinfo;
PGMinfo.geometry.length=length_int;
PGMinfo.geometry.moment_arm=moment_arm_int;

PGMforce_int=zeros(nActs,nFrames);
for iAct=1:nActs
    PGMforce_int(iAct,:)=interpolation_alone(PGMdata.PGMforce(iAct,iSel),nFrames);
end

PGMmoment_int=zeros(nActs,nDOFs,nFrames);
for iAct=1:nActs
    for iDOF=1:nDOFs
        PGMmoment_int(iAct,iDOF,:)=interpolation_alone(squeeze(PGMdata.PGMmoment(iAct,iDOF,iSel)),nFrames);
    end
end

% interpolation_alone(PGMdata.gait_cycle(iSel),nFrames) % verify
PGMdata_int.PGMinfo   =PGMinfo;
PGMdata_int.PGMforce  =PGMforce_int;
PGMdata_int.PGMmoment =PGMmoment_int;
end

function [DatStore_PGMs]=read_muscle_analysis_for_PGMs(Misc,MuscleAnalysisPath_PGMs)
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

function [MC_parameter_BH04,muscle_metRate,Edot_normal_TS,Edot_normal_mean] = computeMetabolicCost(R,Misc)

Results   =R.Results;

iSel      =1:R.nFrames;
sub_mass  =Misc.subject_mass;       % or modelmass = getModelMass(Misc.model_path);

% Order results
% [musT_param,musE_param,mus_time,mus_dyn,~] = Results_states_params(Results,subject_mass,subject_height,extra_frame);

musT_param=Results.MParam;
musE_param=Results.EParam;
mus_time  =Results.time;
mus_dyn   =Results.dyn_val;

nMuscles  =size(musT_param,2);

% compute
mode_basalOn=0; % Boolean: Select if you want to add a basal heat
mode_negWork=1; % Boolean: Select if you want negative heat to be dissipated, thus net energy rate is >=0

muscle_metRate=zeros(nMuscles,length(iSel));
for iMus=1:nMuscles
    mus_Dynamics= [mus_time(:)'; squeeze(mus_dyn(:,iMus,:))];
    [MC_parameter_BH04,~,~,~]= MC_BH04_R(musT_param(:,iMus),musE_param(:,iMus),mus_Dynamics,mode_negWork,mode_basalOn);
    muscle_metRate(iMus,:)= MC_parameter_BH04(1,:)/sub_mass; % normalized by mass
end
Edot_normal_TS   =sum(muscle_metRate); % normalized by mass
Edot_normal_mean =mean(Edot_normal_TS)*2; % 2 legs
% aim_main         =Edot_normal_mean;
end

function [R_int] = interpolatedMRS(R,nFrames)

extra_frame    = R.Misc.extra_frame;
subject_mass   = R.Misc.subject_mass;
subject_height = R.Misc.subject_height;

Results=R.Results;

[musT_param,musE_param,mus_time,mus_dyn,states_field] = Results_states_params(Results,subject_mass,subject_height,extra_frame);

nStates=length(states_field);
nMuscles=size(musT_param,2);

mus_dyn_int =zeros(nStates,nMuscles,nFrames);
for iState=1:nStates
    for iMus=1:nMuscles
        mus_dyn_int(iState,iMus,:) =interpolation_alone(squeeze(mus_dyn(iState,iMus,:)),nFrames);
    end
end
mus_time_int=interpolation_alone(mus_time,nFrames);

R_int.nFrames = nFrames;

R_int.Results.time     =mus_time_int;
R_int.Results.dyn_val  =mus_dyn_int;
R_int.Results.dyn_label=states_field;
R_int.Results.MParam   =musT_param;
R_int.Results.EParam   =musE_param;

% compute additional information
% Moment arms
nDOFs   =R.DatStore.nDOF;
nMuscles=R.DatStore.nMuscles;

MA_int  = zeros(nMuscles,nDOFs*2,nFrames);
IK_int  = zeros(nDOFs,nFrames);
ID_int  = zeros(nDOFs,nFrames);

DofNames_Input=R.Misc.DofNames_Input;
DofNames_Input=removeLastTwoChars(DofNames_Input);

MA_label=cell(1,nDOFs*2);
for iDOF=1:nDOFs
    for iMus=1:nMuscles
        MA=interpolation_alone(R.DatStore.MAinterp((1+extra_frame:end-extra_frame),(iDOF-1)*nMuscles+iMus),nFrames);
        MA_p=zeros(1,nFrames);  MA_n=zeros(1,nFrames);
        for i=1:nFrames
            if MA(1,i)>=0; MA_p(1,i)= MA(1,i); end
            if MA(1,i) <0; MA_n(1,i)=-MA(1,i); end
        end
        MA_int(iMus,(iDOF-1)*2+1,:)=MA_p(:);
        MA_int(iMus,iDOF*2,:)      =MA_n(:);
    end
    IK_int(iDOF,:)=interpolation_alone(R.DatStore.q_exp((1+extra_frame:end-extra_frame),iDOF),nFrames);
    ID_int(iDOF,:)=interpolation_alone(R.DatStore.T_exp((1+extra_frame:end-extra_frame),iDOF),nFrames);

    % Convention for Rajagopal model
    if strcmp(DofNames_Input{iDOF},'hip_flexion')
        label_pos='hip_flexion'; label_neg='hip_extension';
    elseif strcmp(DofNames_Input{iDOF},'hip_adduction')
        label_pos='hip_adduction'; label_neg='hip_abduction';
    elseif strcmp(DofNames_Input{iDOF},'hip_rotation')
        label_pos='hip_rotation_int'; label_neg='hip_rotation_ext';
    elseif strcmp(DofNames_Input{iDOF},'knee_angle')
        label_pos='knee_flexion'; label_neg='knee_extension';
    elseif strcmp(DofNames_Input{iDOF},'ankle_angle')
        label_pos='ankle_dorsiflexion'; label_neg='ankle_plantarflexion';
    end
    MA_label((iDOF-1)*2+1)={label_pos};
    MA_label(iDOF*2)={label_neg};
end

R_int.IK=IK_int;
R_int.ID=ID_int;
R_int.MA=MA_int;

R_int.MA_label=MA_label;
end

function newNames = removeLastTwoChars(names)
    newNames = cell(size(names));
    for i = 1:length(names)
        str = names{i};
        newNames{i} = str(1:end-2);
    end
end
function [Vi]= interpolation_alone(signal,nFrames)            
idx            = 1:length(signal);                      % Index
idxq           = linspace(min(idx), max(idx), nFrames); % Interpolation Vector to nFrames
Vi             = interp1(idx, signal, idxq, 'pchip');   % linear is faster
end

%
% Result_int =mus_dyn_array;
% MuscleNames=R.Results.MuscleNames;
% 
% Label_int={'Time' 'MExcitation' 'MActivation' 'lMtildeopt' 'vMtilde' 'FMltilde' 'FMvtilde' 'TForce' 'Fpe' 'Fce' 'Vce' 'Wce'}; % vMtilde is normalized & Fpe is multiplier
% dir_int=R.Misc.OutPath;

% save(fullfile(R.Misc.OutPath,[R.Misc.OutName '_int.mat']),'Result_int','Label_int','MuscleNames','ADD_MA_stored','ADD_IK_stored','ADD_ID_stored');
% end