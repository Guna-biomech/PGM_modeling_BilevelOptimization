function [J] = computeMetric(Results,musT_param,musE_param,mus_time,mus_dyn,states_field,Exoskeleton_aim,J_normalized)
% Exoskeleton strategy
% Options:
% Aim:
% Exoskeleton_aim=9; % 1Energy 9Muscle

frame_extra=5;
fSel=1+frame_extra:size(Results.MActivation.genericMRS,2)-1-frame_extra;
data_length=length(fSel);

if strcmp(Exoskeleton_aim,'a')
    muscle_sel=Results.MuscleNames; % {'soleus_r' 'gasmed_r' 'gaslat_r'};
    mus_ind   =ismember(Results.MuscleNames,muscle_sel); % mus_ind=find(strcmp(Results.MuscleNames,muscle_sel));
    
    state_sel='MActivation'; % TForce Vce Fce Wdotce 
    state_ind=strcmp(states_field,state_sel); % state_ind=find(strcmp(states_field(2:end),state_sel));
    
    NMuscles=length(Results.MuscleNames);
    main_obs=sum(squeeze(mus_dyn(state_ind,mus_ind,fSel)))/NMuscles;

    NDof      =size(Results.RActivation.genericMRS,1);
    helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)

    aim_main  =sum(main_obs,'all');
    aim_helper=sum(helper_obs,'all');

    J=aim_main+0.01*aim_helper;

elseif strcmp(Exoskeleton_aim,'T')
    % dof=DofExoind; % NO NEED

    NMuscles=length(Results.MuscleNames);

    dof_tot=size(Results.MAinterp,2)/NMuscles;

    MMuscles_sumed=zeros(dof_tot,data_length);
    Mres          =zeros(dof_tot,data_length);
    for i=1:dof_tot
        MAinterp=Results.MAinterp(fSel,(i-1)*(NMuscles)+1:(i*NMuscles));
        TForce  =Results.TForce.genericMRS(:,fSel);
        MMuscles=sum(MAinterp'.*TForce);

        MMuscles_sumed(i,:)=MMuscles;
        Mres(i,:)          =Results.RActivation.genericMRS(i,fSel);
    end

    % % PREVIOUS - based on some DOFs
    % dof=DofExoind;
    % 
    % fSel=1:size(Results.MActivation.genericMRS,2)-1;
    % NMuscles=length(Results.MuscleNames);
    % MAinterp=Results.MAinterp(fSel,(dof-1)*(NMuscles)+1:(dof*NMuscles));
    % TForce  =Results.TForce.genericMRS(:,fSel);
    % MMuscles=MAinterp'.*TForce;
    % MMuscles_sumed=sum(MMuscles);
    % 
    % Mres=Results.RActivation.genericMRS(dof,fSel);

    % MMuscles_sumed_musGroup=MMuscles_sumed((MMuscles_sumed<=0));
    J=sum(abs(MMuscles_sumed+Mres),'all');

elseif strcmp(Exoskeleton_aim,'E')
    % metabolic energy model: output
    mode_basalOn=0; % Boolean: Select if you want to add a basal heat
    mode_negWork=1; % Boolean: Select if you want negative heat to be dissipated, thus net energy rate is >=0
    
    muscle_metRate=double.empty;
    for mus_opt=1:length(Results.MuscleNames)
        mus_Dynamics= [mus_time(fSel); squeeze(mus_dyn(:,mus_opt,fSel))];
       [MC_parameter_BH04,~,~,~]= MC_BH04_R(musT_param,musE_param,mus_Dynamics,mode_negWork,mode_basalOn);
       muscle_metRate(mus_opt,:)=MC_parameter_BH04(1,:);
    end
    oneLeg_metRate=sum(muscle_metRate); % plot(oneLeg_metRate/subjectMass)

    aim_main=sum(oneLeg_metRate)/length(Results.MuscleNames);

    NDof      =size(Results.RActivation.genericMRS,1);
    helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)
    aim_helper=sum(helper_obs,'all');

    J=aim_main+aim_helper;

elseif strcmp(Exoskeleton_aim,'E_UM10')
    % metabolic energy model: output
    mode_basalOn=0; % Boolean: Select if you want to add a basal heat
    mode_negWork=1; % Boolean: UM10 IS BY DEFAULT 1!
    
    muscle_metRate=double.empty;
    for mus_opt=1:length(Results.MuscleNames)
        mus_Dynamics= [mus_time(fSel); squeeze(mus_dyn(:,mus_opt,fSel))];
       [MC_parameter_UM10,~,~,~]= MC_UM10_R(musT_param,musE_param,mus_Dynamics,mode_negWork,mode_basalOn);
       muscle_metRate(mus_opt,:)=MC_parameter_UM10(1,:);
    end
    oneLeg_metRate=sum(muscle_metRate); % plot(oneLeg_metRate/subjectMass)

    aim_main=sum(oneLeg_metRate)/length(Results.MuscleNames);

    NDof      =size(Results.RActivation.genericMRS,1);
    helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)
    aim_helper=sum(helper_obs,'all');

    J=aim_main+aim_helper;

elseif strcmp(Exoskeleton_aim,'E_LU24')
    % metabolic energy model: output
    mode_basalOn=0; % Boolean:
    mode_negWork=1; % Boolean: 
    
    muscle_metRate=double.empty;
    for mus_opt=1:length(Results.MuscleNames)
        mus_Dynamics= [mus_time(fSel); squeeze(mus_dyn(:,mus_opt,fSel))];
       [MC_parameter_LU24,~,~,~]= MC_LU24_R(musT_param,musE_param,mus_Dynamics,mode_negWork,mode_basalOn);
       muscle_metRate(mus_opt,:)=MC_parameter_LU24(1,:);
    end
    oneLeg_metRate=sum(muscle_metRate); % plot(oneLeg_metRate/subjectMass)

    aim_main=sum(oneLeg_metRate)/length(Results.MuscleNames);

    NDof      =size(Results.RActivation.genericMRS,1);
    helper_obs=sum(abs(Results.RActivation.genericMRS(:,fSel)))/NDof; % RA can be possitive (agonist) or negative (antagonist)
    aim_helper=sum(helper_obs,'all');

    J=aim_main+aim_helper;

end

if J_normalized==0
else
    J=(J-J_normalized)/J_normalized*100;
end

end

% function [J] = computeMetric(Results,musT_param,musE_param,mus_time,mus_dyn,states_field,DofExoind,Exoskeleton_aim,J_normalized)
% % Exoskeleton strategy
% % Options:
% % Aim:
% % Exoskeleton_aim=9; % 1Energy 9Muscle
% 
% if Exoskeleton_aim=='a'
%     muscle_sel=Results.MuscleNames; % {'soleus_r' 'gasmed_r' 'gaslat_r'};
%     mus_ind   =ismember(Results.MuscleNames,muscle_sel); % mus_ind=find(strcmp(Results.MuscleNames,muscle_sel));
% 
%     state_sel='MActivation'; % TForce Vce Fce Wdotce 
%     state_ind=strcmp(states_field,state_sel); % state_ind=find(strcmp(states_field(2:end),state_sel));
% 
%     NMuscles=length(Results.MuscleNames);
%     main_obs=sum(squeeze(mus_dyn(state_ind,mus_ind,:)))/NMuscles;
% 
%     NDof      =size(Results.RActivation.genericMRS,1);
%     helper_obs=sum(abs(Results.RActivation.genericMRS))/NDof; % RA can be possitive (agonist) or negative (antagonist)
% 
%     aim_main  =sum(main_obs,'all');
%     aim_helper=sum(helper_obs,'all');
% 
%     J=aim_main+0.01*aim_helper;
% 
% elseif Exoskeleton_aim=='T'
%     dof=DofExoind; % NO NEED
% 
%     fSel=1:size(Results.MActivation.genericMRS,2)-1;
%     NMuscles=length(Results.MuscleNames);
% 
%     dof_tot=size(Results.MAinterp,2)/NMuscles;
% 
%     MMuscles_sumed=zeros(dof_tot,fSel(end));
%     Mres          =zeros(dof_tot,fSel(end));
%     for i=1:dof_tot
%         MAinterp=Results.MAinterp(fSel,(i-1)*(NMuscles)+1:(i*NMuscles));
%         TForce  =Results.TForce.genericMRS(:,fSel);
%         MMuscles=sum(MAinterp'.*TForce);
% 
%         MMuscles_sumed(i,:)=MMuscles;
%         Mres(i,:)          =Results.RActivation.genericMRS(i,fSel);
%     end
% 
%     % % PREVIOUS - based on some DOFs
%     % dof=DofExoind;
%     % 
%     % fSel=1:size(Results.MActivation.genericMRS,2)-1;
%     % NMuscles=length(Results.MuscleNames);
%     % MAinterp=Results.MAinterp(fSel,(dof-1)*(NMuscles)+1:(dof*NMuscles));
%     % TForce  =Results.TForce.genericMRS(:,fSel);
%     % MMuscles=MAinterp'.*TForce;
%     % MMuscles_sumed=sum(MMuscles);
%     % 
%     % Mres=Results.RActivation.genericMRS(dof,fSel);
% 
%     % MMuscles_sumed_musGroup=MMuscles_sumed((MMuscles_sumed<=0));
%     J=sum(abs(MMuscles_sumed+Mres),'all');
% 
% elseif Exoskeleton_aim=='E'
%     % metabolic energy model: output
%     mode_basalOn=0; % Boolean: Select if you want to add a basal heat
%     mode_negWork=1; % Boolean: Select if you want negative heat to be dissipated, thus net energy rate is >=0
% 
%     muscle_metRate=double.empty;
%     for mus_opt=1:length(Results.MuscleNames)
%         mus_Dynamics= [mus_time; squeeze(mus_dyn(:,mus_opt,:))];
%        [MC_parameter_BH04,~,~,~]= MC_BH04_R(musT_param,musE_param,mus_Dynamics,mode_negWork,mode_basalOn);
%        muscle_metRate(mus_opt,:)=MC_parameter_BH04(1,:);
%     end
%     oneLeg_metRate=sum(muscle_metRate); % plot(oneLeg_metRate/subjectMass)
% 
%     aim_main=sum(oneLeg_metRate)/length(Results.MuscleNames);
% 
%     NDof      =size(Results.RActivation.genericMRS,1);
%     helper_obs=sum(abs(Results.RActivation.genericMRS))/NDof; % RA can be possitive (agonist) or negative (antagonist)
%     aim_helper=sum(helper_obs,'all');
% 
%     J=aim_main+aim_helper;
% end
% 
% if J_normalized==0
% else
%     J=(J-J_normalized)/J_normalized*100;
% end
% 
% end