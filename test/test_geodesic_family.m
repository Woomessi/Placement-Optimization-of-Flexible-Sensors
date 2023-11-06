clear
clc
% close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

% 机器人连杆参数
size_sim = 3000; % 蒙特卡洛模拟次数
size_theta_object = 25; % 检测目标圆柱坐标角度均分数

%%%%%%%%%%%%%%%%%
%%% 机器人定义 %%%
%%%%%%%%%%%%%%%%%

% 连杆三角面片模型输入
% TR0 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link0.stl');
TR0 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link0.stl');
X0 = TR0.Points';
X0 = [X0;ones(1,size(X0,2))];
F0 = TR0.ConnectivityList;

% TR1 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link1.stl');
TR1 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link1.stl');
X1 = TR1.Points';
X1 = [X1;ones(1,size(X1,2))];
F1 = TR1.ConnectivityList;

% TR2 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link2.stl');
TR2 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link2.stl');
X2 = TR2.Points';
X2 = [X2;ones(1,size(X2,2))];
F2 = TR2.ConnectivityList;

% TR3 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link3.stl');
TR3 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link3.stl');
X3 = TR3.Points';
X3 = [X3;ones(1,size(X3,2))];
F3 = TR3.ConnectivityList;

% TR4 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link4.stl');
TR4 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link4.stl');
X4 = TR4.Points';
X4 = [X4;ones(1,size(X4,2))];
F4 = TR4.ConnectivityList;

% TR5 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link5.stl');
TR5 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link5.stl');
X5 = TR5.Points';
X5 = [X5;ones(1,size(X5,2))];
F5 = TR5.ConnectivityList;

% TR6 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link6.stl');
TR6 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link6.stl');
X6 = TR6.Points';
X6 = [X6;ones(1,size(X6,2))];
F6 = TR6.ConnectivityList;

% TR7 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link7.stl');
TR7 = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link7.stl');
X7 = TR7.Points';
X7 = [X7;ones(1,size(X7,2))];
F7 = TR7.ConnectivityList;

X_link = {X0,X1,X2,X3,X4,X5,X6};
F_link = {F0,F1,F2,F3,F4,F5,F6};
TR_link = {TR0,TR1,TR2,TR3,TR4,TR5,TR6};

% 机器人生成
my_robot = importrobot('panda_arm.urdf');
size_joint = 7; % 机器人关节数

% 机器人关节角约束，单位：rad
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

% load("edge_candidate_all2.mat")
% load("edge_candidate7.mat")
% edge_candidate_all1{1,7} = edge_candidate;
% for i = 1:size_joint
%     size_edge_candidate(i) = size(edge_candidate_all1{1,i},1);
% end

% load("matlab0601-1.mat")
% load("matlab0601-2.mat")
load("matlab0602.mat")
size_edge_candidate = zeros(1,size_joint);
for i = 1:size_joint
    size_edge_candidate(i) = size(edge_candidate_all{1,i},1);
end
idx_helix = [randi([1,size_edge_candidate(1)]),randi([1,size_edge_candidate(2)]),randi([1,size_edge_candidate(3)]),randi([1,size_edge_candidate(4)]),randi([1,size_edge_candidate(5)]),randi([1,size_edge_candidate(6)]),randi([1,size_edge_candidate(7)])]; % 选择当前传感器布局方案

%%%%%%%%%%%%%%%%%%%%%
%%% 当前关节角配置 %%%
%%%%%%%%%%%%%%%%%%%%%

q = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
link_name = ["panda_link0" "panda_link1" "panda_link2" "panda_link3" "panda_link4" "panda_link5" "panda_link6" "panda_link7"];
config = homeConfiguration(my_robot); % 关节空间配置结构体生成
for i = 1:size_joint % 遍历每个关节
    config(i).JointPosition = q(1,i);
end

%%%%%%%%%%%%%%%%%%%%%%%
%%% 各连杆传感器配置 %%%
%%%%%%%%%%%%%%%%%%%%%%%

% for idx_link = 1:size_joint % 遍历所有连杆 (末端连杆对应空矩阵)
  for idx_link = 3
    transform = getTransform(my_robot,config,link_name(idx_link)); %当前位形空间下连杆相对于基坐标系的齐次变换矩阵
    X_update = transform*X_link{1,idx_link}; % 当前时刻连杆的mesh坐标
    F = (double(F_link{1,idx_link}))'; % 当前连杆mesh的三角面片信息
    size_F = size(F,2); % 三角面数量
    X = (double(X_update(1:3,:))); % 当前连杆mesh的顶点坐标信息
    size_X = size(X,2); % 三角面片顶点数量
    TR = triangulation(F',X'); % MATLAB官方的三角面片数据结构

    %%%%%%%%%%%%%%%%%%%
    %%% Heat Method %%%
    %%%%%%%%%%%%%%%%%%%

    % pstart = edge_candidate_all{1,idx_link}(idx_helix(idx_link),1); % 起点序号
    pstart = 3823;
    % pstart = 19765;
    phi = getGeodesicDistanceHeat(X,F,pstart);
 
    %%%%%%%%%%%%%%%%%%%%%
    %%% Fast Marching %%%
    %%%%%%%%%%%%%%%%%%%%%

    % pstart = edge_candidate_all{1,idx_link}(idx_helix(idx_link),1); % 起点序号
    % phi = perform_fast_marching_mesh(X, F, pstart);

    %%%%%%%%%%%%%%%%%%%%%
    %%% 计算测地线路径 %%%
    %%%%%%%%%%%%%%%%%%%%%

    % pend = edge_candidate_all{1,idx_link}(idx_helix(idx_link),2); % 终点序号
    pend = 19765;
    % pend = 3823;
    paths = compute_geodesic_mesh(phi, X, F, pend);
    paths = unique(paths',"rows","stable");
    paths = paths';

    %%%%%%%%%%%%
    %%% 作图 %%%
    %%%%%%%%%%%%

    plot_fast_marching_mesh(X, F, phi, paths);
  end