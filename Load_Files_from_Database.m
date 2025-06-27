%% Load database
clc; clf;
dirOutput='C:\Users\movea\Dropbox\Collaboration\Guna';

sub_list=[1:5];
act_list=[1:3];
gc_list =[1:3];

plot_sel='IK'; % IK or ID

if strcmp(plot_sel,'IK')
    name_ext_label='';          extension='.mot';     hard_code_ind=9;
    axis_lim ={[-20 50]      [-15 15]       [-15 15]      [-10 80]    [-30 20]};
else %strcmp(plot_sel,'ID')
    name_ext_label='_moment';   extension='.sto';     hard_code_ind=7;
    axis_lim ={[-2 2]        [-2 2]         [-2 2]        [-2 2]      [-2 2]};
end

joint_names  ={'hip_flexion_' 'hip_adduction_' 'hip_rotation_' 'knee_angle_' 'ankle_angle_'};

for sub_opt=1:length(sub_list)
    sub_sel=sub_list(sub_opt);
    subInfo=load(fullfile(dirOutput,'database',['sub' num2str(sub_sel)],'model',['subject_information_sub' num2str(sub_sel) '.mat']));
    mass    =subInfo.subject_info.mass;
    if strcmp(plot_sel,'IK'); normalization=1; elseif strcmp(plot_sel,'ID'); normalization=mass; end

    for act_opt=1:length(act_list)
        act_sel=act_list(act_opt);
        for gc_opt=1:length(gc_list)
            gc_sel=gc_list(gc_opt);

            gaitData=load(fullfile(dirOutput,'database',['sub' num2str(sub_sel)],'gaitData',['gaitFeatureData_v' num2str(act_sel) '_t' num2str(gc_sel) '.mat']));
            side    =gaitData.gaitData.side;

            data=importdata(fullfile(dirOutput,'database',['sub' num2str(sub_sel)],plot_sel,[plot_sel '_sub' num2str(sub_sel) '_v' num2str(act_sel) '_t' num2str(gc_sel) extension]));
            data_headers=data.textdata(hard_code_ind,:);
            data_values =data.data/normalization;

            data_length=length(data_values);
            gait_cycle =linspace(0,100,data_length);
            for i=1:length(joint_names)
                subplot(3,length(joint_names),i+(act_opt-1)*length(joint_names))
                joint_sel=data_values(:,strcmp(data_headers,[joint_names{i} side name_ext_label]));
                hold on;
                plot(gait_cycle,joint_sel);
                ylim(axis_lim{i})
            end
        end
    end
end