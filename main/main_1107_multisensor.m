clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

tic
%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%
% 仿真参数
size_sim = 3000; % 蒙特卡洛模拟次数
size_theta_object = 25; % 检测目标圆柱坐标角度均分数

% 传感器参数
size_spot = 16;
l_sensor = 0.27; % （单位：米）柔性传感器长度
% tol_l_sensor = 0.001; % 柔性传感器长度容限

% ToF模块参数
fov_horizontal = 25; % （单位：度）水平视场角
fov_vertical = 25; % （单位：度）垂直视场角
range_max = 0.5; % （单位：米）最大检测距离
h_cone = range_max*cosd(fov_horizontal/2); % （单位：米）视场锥高度

%%%%%%%%%%%%%%%%%
%%% 机器人定义 %%%
%%%%%%%%%%%%%%%%%

% 机器人生成
% my_robot = importrobot('panda_arm.urdf');
my_robot = importrobot("frankaEmikaPanda.urdf");

size_joint = 7; % 机器人关节数

% （单位：弧度），机器人关节角约束，
joint_constraint = zeros(2,size_joint);
joint_constraint(:,1) = [-2.8973;2.8973];
joint_constraint(:,2) = [-1.7628;1.7628];
joint_constraint(:,3) = [-2.8973;2.8973];
joint_constraint(:,4) = [-3.0718;-0.0698];
joint_constraint(:,5) = [-2.8973;2.8973];
joint_constraint(:,6) = [-0.0175;3.7525];
joint_constraint(:,7) = [-2.8973;2.8973];

%%%%%%%%%%%%%%%%%%%
%%% 检测目标定义 %%%
%%%%%%%%%%%%%%%%%%%

% createObject参数：半径（米）、高度（米，取中国女性平均身高）、圆柱坐标均分数
[size_point_target, target] = createObject(0.149, 1.57, size_theta_object); % 检测目标点生成
target_homo = [target;ones(1,size_point_target)]; % 目标点的齐次坐标
r_min = 0;
r_max = 0.855;
r_obj_offset = r_min+0.149+(r_max-r_min)*rand(size_sim,1); % （单位：米）相对于机器人的距离随机偏移量

% % 随机方位
% theta_obj_offset = 360*rand(size_sim,1); % （单位：度）相对于机器人的角度随机偏移量

% 任务方位
ang_min = -90;
ang_max = 45;
theta_obj_offset = ang_min+(ang_max-ang_min)*rand(size_sim,1); % （单位：度）相对于机器人的角度随机偏移量

%%%%%%%%%%%%%%%%%%%%%
%%% 关节角随机配置 %%%
%%%%%%%%%%%%%%%%%%%%%

% % 随机轨迹
% q_all = zeros(size_sim,size_joint);
% for idx_config = 1:size_sim
%     q_all(idx_config,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
% end

% 任务轨迹
load("interpState2.mat",'interpStates')
q_all = [interpStates;flip(interpStates(1:39,:))];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 输入各关节所有测地线组合 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

load("all_tform_spot_link6.mat",'all_tform_spot');
all_tform_spot_link6 = all_tform_spot;
size_all_tform_spot_link6 = size(all_tform_spot_link6,2);

% idx_link = [1,2];
% idx_geodesic = [18595,13900];

% idx_link = 1;
% idx_geodesic = 18595;

idx_link = 0:6;
idx_geodesic = [6929, 1142, 20761, 10200, 10451, 13100, 17];

% 提取当前连杆TOF模块的齐次变换矩阵
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

% DH参数
a = [0;0;0;0.0825;-0.0825;0;0.088;0];
d = [0.333;0;0.316;0;0.384;0;0;0.107];
alpha = [0;-pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

detection_times = 0; % 检测到目标的次数
for idx_config = 1:5:size_sim

    % 检测目标坐标信息
    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; % 检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; % 平移变换矩阵
    target_homo_updated = tform_target*target_homo;
    target_updated = target_homo_updated(1:3,:); % 平移变换后的检测目标

    % 利用余数对机械臂的任务轨迹进行循环
    if mod(idx_config,size(q_all,1)) == 0
        idx_q = size(q_all,1);
    else
        idx_q = mod(idx_config,size(q_all,1));
    end

    % 当前关节角配置
    q = q_all(idx_q,1:7); % 当前关节空间角
    config = homeConfiguration(my_robot); % 关节空间配置结构体生成
    for idx_joint = 1:size_joint % 遍历每个关节
        config(idx_joint).JointPosition = q(1,idx_joint);
    end

    show(my_robot,config); % 打开figure属性
    hold on
    scatter3(target_updated(1,:),target_updated(2,:),target_updated(3,:),2,"magenta","filled")
    % view(90,20)
    view(0,90)
    xlim([-1 1])
    ylim([-1 1])
    zlim([0 1.5])
    for idx_round = 1:size_sensor % 遍历所有连杆 (idx_link = 1 对应 link0，末端连杆对应空矩阵)
        idx_link_current = idx_link(idx_round);
        % 对各连杆上的传感器进行配置
        % 当前位形空间下当前连杆相对于基坐标系的齐次变换矩阵
        % link_name = ["panda_link0" "panda_link1" "panda_link2" "panda_link3" "panda_link4" "panda_link5" "panda_link6" "panda_link7"];
        % transform = getTransform(my_robot,config,link_name(idx_link));

        transform = eye(4);
        if idx_link_current == 0
            transform = eye(4);
        else
            for i = 1:idx_link_current
                transform = transform*getTformMDH(a(i),d(i),alpha(i),q(i));
            end
        end

        % 选择当前连杆的传感器布局方案
        all_tform_spot = group_all_tform_spot{1,idx_round};
        tform_spot = all_tform_spot{1,idx_geodesic(idx_round)};

        % 通过齐次变换获得当前位形空间下的传感器ToF模块位姿
        for idx_spot = 1:size_spot
            tform_spot{1,idx_spot} = transform*tform_spot{1,idx_spot};
        end

        % 作图
        plotRobotMultisensor(my_robot,config,size_spot,tform_spot,h_cone,fov_horizontal,target_updated);
    end

    % 对各连杆上的传感器进行配置
    for idx_round = 1:size_sensor % 遍历所有连杆 (idx_link = 1 对应 link0，末端连杆对应空矩阵)
        idx_link_current = idx_link(idx_round);
        % 对各连杆上的传感器进行配置
        % 当前位形空间下当前连杆相对于基坐标系的齐次变换矩阵
        % link_name = ["panda_link0" "panda_link1" "panda_link2" "panda_link3" "panda_link4" "panda_link5" "panda_link6" "panda_link7"];
        % transform = getTransform(my_robot,config,link_name(idx_link));

        transform = eye(4);
        if idx_link_current == 0
            transform = eye(4);
        else
            for i = 1:idx_link_current
                transform = transform*getTformMDH(a(i),d(i),alpha(i),q(i));
            end
        end

        % 选择当前连杆的传感器布局方案
        all_tform_spot = group_all_tform_spot{1,idx_round};
        tform_spot = all_tform_spot{1,idx_geodesic(idx_round)};

        % 通过齐次变换获得当前位形空间下的传感器ToF模块位姿
        for idx_spot = 1:size_spot
            tform_spot{1,idx_spot} = transform*tform_spot{1,idx_spot};
        end

        % 作图
        % plotRobotMultisensor(my_robot,config,size_spot,tform_spot,h_cone,fov_horizontal,target_updated);

        % ToF检测
        for idx_point_target = 1:size_point_target
            flag_successful_detection = 0; %循环跳出标识
            point_target = target_updated(:,idx_point_target); %目标点设置
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot{1,idx_spot};
                if tform_spot_current(:,1) == zeros(4,1)
                    break;
                end
                vt = point_target - tform_spot_current(1:3,4); %圆锥顶点到目标点的向量
                l_vt = norm(vt);
                centerline = tform_spot_current(1:3,1); %圆锥中心线
                cos_theta = dot(vt,centerline)/(l_vt*norm(centerline)); %夹角余弦
                if cos_theta > cosd(min(fov_vertical,fov_horizontal)/2) %夹角是否小于视场角的一半？
                    if l_vt*cos_theta < h_cone %测距值在中心线方向的投影距离是否在量程内
                        % range_all(idx_spot, idx_point_target) = norm(vt);
                        % range_old(idx_spot, 1) = range_now(idx_spot, 1);
                        % if l_vt < range_old(idx_spot, 1)
                        %     range_now(idx_spot, 1) = l_vt;
                        % end
                        flag_successful_detection = 1;
                        detection_times = detection_times + 1;
                        break
                    end
                end
            end
            if flag_successful_detection == 1 %跳出两重循环
                break
            end
        end
        % 作图
        % plotRobot(my_robot,config,size_spot,tform_spot,h_cone, fov_horizontal,target1);
        if flag_successful_detection == 1 %跳出三重循环
            break
        end
    end
    hold off
end
detective_rate = -detection_times/size_sim;
toc