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
size_sim = 6000; % 蒙特卡洛模拟次数
size_theta_object = 25; % 检测目标圆柱坐标角度均分数

% 传感器参数
l_sensor = 0.27; % （单位：米）柔性传感器长度

% ToF模块参数
fov_horizontal = 25; % （单位：度）水平视场角
fov_vertical = 25; % （单位：度）垂直视场角
range_max = 0.5; % （单位：米）最大检测距离
h_cone = range_max*cosd(fov_horizontal/2); % （单位：米）视场锥高度

%%%%%%%%%%%%%%%%%
%%% 机器人定义 %%%
%%%%%%%%%%%%%%%%%

% 机器人生成
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

% 任务方位
ang_min = -90;
ang_max = 45;
theta_obj_offset = ang_min+(ang_max-ang_min)*rand(size_sim,1); % （单位：度）相对于机器人的角度随机偏移量

%%%%%%%%%%%%%%%%%%%%%
%%% 关节角随机配置 %%%
%%%%%%%%%%%%%%%%%%%%%

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

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 只使用1个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = 6;
% idx_geodesic = 17;% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用2个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [0,6];
% idx_geodesic = [6929, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用3个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [0,5,6];
% idx_geodesic = [6929, 13100, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用4个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [0,1,5,6];
% idx_geodesic = [6929, 1142, 13100, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用5个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [0,1,2,5,6];
% idx_geodesic = [6929, 1142, 20761, 13100, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用6个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [0,1,2,4,5,6];
% idx_geodesic = [6929, 1142, 20761, 10451, 13100, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用7个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = 0:6;
% idx_geodesic = [6929, 1142, 20761, 10200, 10451, 13100, 17];% 严格按顺序
% detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

toc