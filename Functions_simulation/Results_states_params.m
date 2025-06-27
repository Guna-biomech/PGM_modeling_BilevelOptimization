function [musT_param,musE_param,mus_time,mus_dyn,states_field] = Results_states_params(Results,subjectMass,subjectHeight,ext_frames)
nMuscles=length(Results.MuscleNames);

% subject: output
V_total_leg    = 47*subjectMass*(subjectHeight/100)+1285; % Input: Kg & meter - Output: cm^3

% muscle-tendon states: output
% - parameters
param_field={'FMo' 'lMo' 'lTs' 'alphao' 'vMmax' 'kpe' 'so' 'sM'};
musT_field ={'FMo' 'lMo' 'vMmax'};
musE_field ={'Mvol' 'Mpcsa' 'Mft' 'Mmass'};

musE_param = NaN(length(musE_field),nMuscles);
musT_param = NaN(length(param_field),nMuscles);

rho        = 1059.7;  % kg/m^3 [Umberger 2003]

ind_musT   =find(contains(param_field,musT_field));
musT_param(ind_musT,:) =Results.params.genericMRS(ind_musT,:); % param x nMuscle

musE_param(1,:) = getVolumeFraction(Results.MuscleNames)/100*(V_total_leg./(100)^3);    % m^3 
musE_param(2,:) = (musE_param(1,:))./(musT_param(2,:));                         % Mvol/lMo
musE_param(3,:) = (1-getSlowTwitchRatios_Upd(Results.MuscleNames));                     % [fraction]
musE_param(4,:) = musE_param(1,:).*rho;                                         % Mvol*rho       
% musE_param(4,:) =0.2662; % antoine mass

% - states     
states_field={'MExcitation' 'MActivation' 'lMtildeopt' 'vMtilde' 'FMltilde' 'FMvtilde' 'TForce' 'Fpe'}; % 'Time' is excluded          
iSel=1+ext_frames:length(Results.Time.genericMRS)-ext_frames-1;
data_length=length(iSel);

mus_time= NaN(1,data_length);
mus_dyn = NaN(length(states_field),nMuscles,data_length);   

Time=Results.Time.genericMRS(iSel);

mus_time(1,:)=Time;
for i=1:length(states_field)
    mus_dyn(i,:,:)=Results.(states_field{i}).genericMRS(:,iSel);
end

% get muscle passive force-length multiplier
mus_dyn(8,:,:)=mus_dyn(8,:,:)./musT_param(1,:);  % FMlpas

% compute muscle force, velocity and work
states_field={states_field{1:end-1} 'FMlpas' 'Fce' 'Vce' 'Wdotce'};         
mus_dyn(9,:,:)=  mus_dyn(2,:,:).*mus_dyn(5,:,:).*mus_dyn(6,:,:).*musT_param(1,:);
mus_dyn(10,:,:)= mus_dyn(4,:,:).*musT_param(2,:);
mus_dyn(11,:,:)=-mus_dyn(10,:,:).*mus_dyn(9,:,:);
end