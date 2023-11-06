clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%
%%% 优化变量 %%%
%%%%%%%%%%%%%%%

idx_link = 5; %传感器布置在哪个连杆上
size_spot = 16; %ToF模块数量
% idx_helix = 30; %选择当前传感器布局方案

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

size_sim = 5000; %蒙特卡洛模拟次数
size_theta = 50; %圆柱坐标角度均分数
size_theta_object = 15; %目标圆柱坐标角度均分数

%机器人连杆参数
h_cylinder = 0.09; %圆柱连杆高度
r_cylinder = 0.043; %圆柱连杆半径

%传感器布局参数
l_sensor = 0.27; %柔性传感器长度
tol_l_sensor = 0.001; %柔性传感器长度容限

%ToF模块参数
fov_horizontal = 25; %水平视场角，单位为degree
fov_vertical = 25; %垂直视场角，单位为degree
range_max = 0.5; %最大测量距离
h_cone = range_max*cosd(fov_horizontal/2); %视场锥高度

%%%%%%%%%%%%%%%%%
%%% 机器人定义 %%%
%%%%%%%%%%%%%%%%%

%机器人生成
% my_robot = importrobot('model_sensor_compact.urdf'); %无mesh，运行更快
% my_robot = importrobot('model_sensor_compact_win.urdf');

% load("my_robot.mat");
% load("my_robot_mesh.mat");

my_robot = loadrobot("frankaEmikaPanda");

size_joint = 7; %机器人关节数

%机器人关节角约束
joint_constraint = zeros(2,size_joint);
joint_constraint(:,1) = [-2.8973;2.8973];
joint_constraint(:,2) = [-1.7628;1.7628];
joint_constraint(:,3) = [-2.8973;2.8973];
joint_constraint(:,4) = [-3.0718;-0.0698];
joint_constraint(:,5) = [-2.8973;2.8973];
joint_constraint(:,6) = [-0.0175;3.7525];
joint_constraint(:,7) = [-2.8973;2.8973];

%%%%%%%%%%%%%%
%%% DH参数 %%%
%%%%%%%%%%%%%%

a = [0;0;0;0.0825;-0.0825;0;0.088;0];
d = [0.333;0;0.316;0;0.384;0;0;0.107];
alpha = [0;-pi/2;pi/2;pi/2;-pi/2;pi/2;pi/2;0];

%%%%%%%%%%%%%%%%%%%%%%%
%%% 离散化连杆圆柱面 %%%
%%%%%%%%%%%%%%%%%%%%%%%

[size_point, point_all] = discretizeCylinder(r_cylinder, h_cylinder, size_theta);

%%%%%%%%%%%%%%%%%%%%%
%%% 测地线生成 %%%
%%%%%%%%%%%%%%%%%%%%%

[edge, size_edge, l_edge] = getHelix(size_point, point_all, r_cylinder);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 提取与传感器长度相近的测地线 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[edge_candidate, size_edge_candidate] = pickHelix(tol_l_sensor, size_edge, l_edge, l_sensor);

%%%%%%%%%%%%%%%%%%%
%%% 检测目标配置 %%%
%%%%%%%%%%%%%%%%%%%

% [size_point_target, target] = createObject(0.149, 1.57, size_theta_object); %检测目标点生成
% target_homo = [target;ones(1,size_point_target)]; %检测目标的齐次坐标
% translation = [0.5;0;0]; %检测目标平移向量
% tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
% target_homo = tform_target*target_homo;
% target = target_homo(1:3,:); %平移变换后的检测目标

[size_point_target, target] = createObject(0.149, 1.57, size_theta_object); %检测目标点生成
target_homo = [target;ones(1,size_point_target)]; %检测目标的齐次坐标
r_obj_offset = 0.27+(0.5-0.27)*rand(size_sim,1);
theta_obj_offset = 360*rand(size_sim,1);

% size_var = 2;
size_var = 1;

% lb = [1,1];
% ub = [16,size_edge_candidate];

lb = 1;
ub = size_edge_candidate;

% intcon = [1,2];
intcon = 1;

options = optimoptions("surrogateopt","Display",'iter',"PlotFcn",'surrogateoptplot',"UseParallel",true);

[solution,objectiveValue] = surrogateopt(@(idx_helix)detective_rate_helix(idx_helix,size_spot,r_obj_offset,theta_obj_offset,target_homo,joint_constraint,a,d,alpha,edge_candidate,point_all,edge, r_cylinder,size_point_target,fov_vertical,fov_horizontal,size_sim,h_cone,idx_link),lb,ub,intcon,options);