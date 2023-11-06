clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

% tic
%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%
% 仿真参数
size_sim = 3000; % 蒙特卡洛模拟次数
size_theta_object = 25; % 检测目标圆柱坐标角度均分数

% 传感器参数
size_spot = 16;
l_sensor = 0.27; % （单位：米）柔性传感器长度
tol_l_sensor = 0.001; % 柔性传感器长度容限

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
my_robot.DataFormat = "row";
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
r_obj_offset = 0.5+0.149+(0.855-0.5)*rand(size_sim,1); % （单位：米）相对于机器人的距离随机偏移量
theta_obj_offset = 360*rand(size_sim,1); % （单位：度）相对于机器人的角度随机偏移量

%%%%%%%%%%%%%%%%%%%%%
%%% 关节角随机配置 %%%
%%%%%%%%%%%%%%%%%%%%%

load("interpState2.mat")
paths = interpStates;
% load("interpState2.mat")
% paths = [paths;interpStates];
config = homeConfiguration(my_robot); % 关节空间配置结构体生成
for i = 1:size(paths,1)
    % for idx_joint = 1:size_joint % 遍历每个关节
    %     config(idx_joint).JointPosition = paths(i,idx_joint);
    % end
    % show(my_robot,config);
    % show(my_robot,paths(i,:));
    show(my_robot, paths(i,:),...
        "PreservePlot", false,...
        "Visuals","on");
    xlim([-1 1])
    ylim([-1 1])
    zlim([0 1.5])
    % hold on
    drawnow
end
for i = 1:size(paths,1)
    % for idx_joint = 1:size_joint % 遍历每个关节
    %     config(idx_joint).JointPosition = paths(i,idx_joint);
    % end
    % show(my_robot,config);
    % show(my_robot,paths(i,:));
    show(my_robot, paths(size(paths,1)+1-i,:),...
        "PreservePlot", false,...
        "Visuals","on");
    % hold on
    xlim([-1 1])
    ylim([-1 1])
    zlim([0 1.5])
    drawnow
end
% % 利用随机数生成关节空间位形
% q_all = zeros(size_sim,size_joint);
% q_all(1,:) = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4];
%
% for idx_config = 1:size_sim
%     delta_angle = pi/8*(1 - cos(pi/2.5*idx_config));
%     q_all(idx_config,:) = [q_all(1,1), q_all(1,2),q_all(1,3), q_all(1,4) + delta_angle, q_all(1,5) + delta_angle, q_all(1,6), q_all(1,7) + delta_angle];
% end
%
% % DH参数
% a = [0;0;0;0.0825;-0.0825;0;0.088;0];
% d = [0.333;0;0.316;0;0.384;0;0;0.107];
% alpha = [0;-pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];
%
% %%%%%%%%%%%%%%%%%%%
% %%% 蒙特卡洛仿真 %%%
% %%%%%%%%%%%%%%%%%%%
%
% detection_times = 0; % 检测到目标的次数
% for idx_config = 1:size_sim
%
%     % % 检测目标坐标信息
%     % translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; % 检测目标平移向量
%     % tform_target = [eye(3),translation;0 0 0 1]; % 平移变换矩阵
%     % target_homo_updated = tform_target*target_homo;
%     % target_updated = target_homo_updated(1:3,:); % 平移变换后的检测目标
%
%     % 当前关节角配置
%     q = q_all(idx_config,:); % 当前关节空间角
%     config = homeConfiguration(my_robot); % 关节空间配置结构体生成
%     for idx_joint = 1:size_joint % 遍历每个关节
%         config(idx_joint).JointPosition = q(1,idx_joint);
%     end
%
%     show(my_robot,config); % 打开figure属性
%     hold on
%     % scatter3(target_updated(1,:),target_updated(2,:),target_updated(3,:),2,"magenta","filled")
%     view(60,20)
%     xlim([-1 1])
%     ylim([-1 1])
%     zlim([0 1.5])
% end