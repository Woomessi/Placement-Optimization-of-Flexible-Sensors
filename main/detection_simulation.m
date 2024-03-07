clear
clc
close all
clear options;
addpath(genpath('C:\Placement-Optimization-of-Flexible-Sensors-release'))

tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Basic Configuration %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
size_sim = 3000; % number of Monte Carlo simulations
size_theta_object = 25; % number of the surface points of the obstacle
 
size_spot = 16; % number of ToF modules
l_sensor = 0.27; % length of the flexible sensor

fov_horizontal = 25; % horizontal fov (degree)
fov_vertical = 25; % vertical fov (degree)
range_max = 0.5; % maximum ranging distance (meter)
h_cone = range_max*cosd(fov_horizontal/2); % height of the fov cone (meter)

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Define the robot %%%
%%%%%%%%%%%%%%%%%%%%%%%%

my_robot = importrobot('panda_arm.urdf');

size_joint = 7; % number of the links

% joint angle constraints (rad)
joint_constraint = zeros(2,size_joint);
joint_constraint(:,1) = [-2.8973;2.8973];
joint_constraint(:,2) = [-1.7628;1.7628];
joint_constraint(:,3) = [-2.8973;2.8973];
joint_constraint(:,4) = [-3.0718;-0.0698];
joint_constraint(:,5) = [-2.8973;2.8973];
joint_constraint(:,6) = [-0.0175;3.7525];
joint_constraint(:,7) = [-2.8973;2.8973];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Define the detection object %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% parameters of createObject：radius (meter), height (meter), number of the surface points
[size_point_target, target] = createObject(0.311/2, 1.75, size_theta_object);
target_homo = [target;ones(1,size_point_target)]; % homogenous coordinates of the obstacle points

% consider the height of the base 
for i = 1:size_point_target
    target_homo(:,i) = [1,0,0,0;0,1,0,0;0,0,1,-0.8;0,0,0,1]*target_homo(:,i);
end

r_min = 0;
r_max = 0.855;
r_obj_offset = r_min+0.149+(r_max-r_min)*rand(size_sim,1); % random distance to the robot (meter)

ang_min = -90;
ang_max = 45;
theta_obj_offset = ang_min+(ang_max-ang_min)*rand(size_sim,1); % random angle to the robot (meter)

%%%%%%%%%%%%%%%%%%%%%%%
%%% Define the task %%%
%%%%%%%%%%%%%%%%%%%%%%%

load("interpState2.mat",'interpStates')
q_all = [interpStates;flip(interpStates(1:39,:))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Input the placement candidates of each link (from "geodesic_generation.m") %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load("all_tform_spot_link0.mat",'all_tform_spot')
all_tform_spot_link0 = all_tform_spot;
size_all_tform_spot_link0 = size(all_tform_spot_link0,2);

load("all_tform_spot_link1.mat",'all_tform_spot');
all_tform_spot_link1 = all_tform_spot;
size_all_tform_spot_link1 = size(all_tform_spot_link1,2);

load("all_tform_spot_link2.mat",'all_tform_spot');
all_tform_spot_link2 = all_tform_spot;
size_all_tform_spot_link2 = size(all_tform_spot_link2,2);

load("all_tform_spot_link3.mat",'all_tform_spot');
all_tform_spot_link3 = all_tform_spot;
size_all_tform_spot_link3 = size(all_tform_spot_link3,2);

load("all_tform_spot_link4.mat",'all_tform_spot');
all_tform_spot_link4 = all_tform_spot;
size_all_tform_spot_link4 = size(all_tform_spot_link4,2);

load("all_tform_spot_link5.mat",'all_tform_spot');
all_tform_spot_link5 = all_tform_spot;
size_all_tform_spot_link5 = size(all_tform_spot_link5,2);

load("all_tform_spot_link6_3.mat",'all_tform_spot');
all_tform_spot_link6 = all_tform_spot;
size_all_tform_spot_link6 = size(all_tform_spot_link6,2);

idx_link = 0:6;
idx_geodesic = [6929, 1142, 20761, 10200, 10451, 13100, 65];

% transformation matrices of the current link's ToF modules
size_sensor = size(idx_link,2);
group_all_tform_spot = cell(1,size_sensor);
for i = 1:size_sensor
    switch idx_link(i)
        case 0
            group_all_tform_spot{1,i} = all_tform_spot_link0;
        case 1
            group_all_tform_spot{1,i} = all_tform_spot_link1;
        case 2
            group_all_tform_spot{1,i} = all_tform_spot_link2;
        case 3
            group_all_tform_spot{1,i} = all_tform_spot_link3;
        case 4
            group_all_tform_spot{1,i} = all_tform_spot_link4;
        case 5
            group_all_tform_spot{1,i} = all_tform_spot_link5;
        case 6
            group_all_tform_spot{1,i} = all_tform_spot_link6;
    end
end

% DH parameters
a = [0;0;0;0.0825;-0.0825;0;0.088;0];
d = [0.333;0;0.316;0;0.384;0;0;0.107];
alpha = [0;-pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Monte Carlo simulation %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

detection_times = 0; % successful detection times
for idx_config = 1:size_sim

    % varying position of the obstacle
    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; % 检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1];
    target_homo_updated = tform_target*target_homo;
    target_updated = target_homo_updated(1:3,:);

    % circle the task trajectory of the robot
    if mod(idx_config,size(q_all,1)) == 0
        idx_q = size(q_all,1);
    else
        idx_q = mod(idx_config,size(q_all,1));
    end

    % current robot configuration
    q = q_all(idx_q,1:7); 
    config = homeConfiguration(my_robot); 
    for idx_joint = 1:size_joint 
        config(idx_joint).JointPosition = q(1,idx_joint);
    end

    % visualization initialization
    show(my_robot,config); 
    hold on
    scatter3(target_updated(1,:),target_updated(2,:),target_updated(3,:),2,"magenta","filled")
    view(0,90)
    xlim([-1 1])
    ylim([-1 1])
    zlim([-0.8 1.2])

    % traverse all the links
    for idx_round = 1:size_sensor 
        idx_link_current = idx_link(idx_round);

        % transformation matrix of the current link related to the base
        % frame
        transform = eye(4);
        if idx_link_current == 0
            transform = eye(4);
        else
            for i = 1:idx_link_current
                transform = transform*getTformMDH(a(i),d(i),alpha(i),q(i));
            end
        end

        % select current link's senso location
        all_tform_spot = group_all_tform_spot{1,idx_round};
        tform_spot = all_tform_spot{1,idx_geodesic(idx_round)};

        % poses of ToF modules under the current robot configuration
        for idx_spot = 1:size_spot
            tform_spot{1,idx_spot} = transform*tform_spot{1,idx_spot};
        end

        % visualization
        plotRobotMultisensor(my_robot,config,size_spot,tform_spot,h_cone,fov_horizontal,target_updated);

        % obstacle detection
        for idx_point_target = 1:size_point_target
            flag_successful_detection = 0; 
            point_target = target_updated(:,idx_point_target);
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot{1,idx_spot};
                if tform_spot_current(:,1) == zeros(4,1)
                    break;
                end
                vt = point_target - tform_spot_current(1:3,4); % vector from the detection cone apex to a obstacle point
                l_vt = norm(vt);
                centerline = tform_spot_current(1:3,1); % centerline of the cone
                cos_theta = dot(vt,centerline)/(l_vt*norm(centerline)); 
                if cos_theta > cosd(min(fov_vertical,fov_horizontal)/2) 
                    if l_vt*cos_theta < h_cone 
                        flag_successful_detection = 1;
                        detection_times = detection_times + 1;
                        break
                    end
                end
            end
            if flag_successful_detection == 1 
                break
            end
        end
        if flag_successful_detection == 1
            break
        end
    end
    hold off
end
detective_rate = -detection_times/size_sim;
toc
