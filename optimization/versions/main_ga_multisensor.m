clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%
%%% 优化变量 %%%
%%%%%%%%%%%%%%%

size_spot = 16; %ToF模块数量

% idx_helix = [2000,2000,2000,2000,2000,2000,2000]; %选择当前传感器布局方案

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

%机器人连杆参数
h_cylinder = [0.09,0.09,0.09,0.09,0.09,0.09,0.09]; %圆柱连杆高度
r_cylinder = [0.043,0.043,0.043,0.043,0.043,0.043,0.043]; %圆柱连杆半径

size_sim = 3000; %蒙特卡洛模拟次数
size_theta = 50; %连杆圆柱坐标角度均分数
size_theta_object = 25; %目标圆柱坐标角度均分数

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

%%%%%%%%%%%%%%%%%%%
%%% 检测目标配置 %%%
%%%%%%%%%%%%%%%%%%%

[size_point_target, target] = createObject(0.149, 1.57, size_theta_object); %检测目标点生成
target_homo = [target;ones(1,size_point_target)]; %检测目标的齐次坐标
r_obj_offset = 0.27+(0.5-0.27)*rand(size_sim,1);
theta_obj_offset = 360*rand(size_sim,1);

%利用随机数生成关节空间配置
q_all = zeros(size_sim,size_joint);
for i = 1:size_sim
    q_all(i,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
end

edge = cell(1,size_joint);
point_all = cell(1,size_joint);
edge_candidate = cell(1,size_joint);
size_edge_candidate = zeros(1,size_joint);

for idx_link = 1:size_joint
    %%%%%%%%%%%%%%%%%%%%%%%
    %%% 离散化连杆圆柱面 %%%
    %%%%%%%%%%%%%%%%%%%%%%%

    [size_point, point_all{1,idx_link}] = discretizeCylinder(r_cylinder(idx_link), h_cylinder(idx_link), size_theta);

    %%%%%%%%%%%%%%%%%%%%%
    %%% 测地线生成 %%%
    %%%%%%%%%%%%%%%%%%%%%

    [edge{1,idx_link}, size_edge, l_edge] = getHelix(size_point, point_all{1,idx_link}, r_cylinder(idx_link));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% 提取与传感器长度相近的测地线 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [edge_candidate{1,idx_link}, size_edge_candidate(idx_link)] = pickHelix(tol_l_sensor, size_edge, l_edge, l_sensor);
end

% detective_rate_multisensor(idx_helix(1), idx_helix(2), idx_helix(3), idx_helix(4), idx_helix(5), idx_helix(6), idx_helix(7), size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, size_joint, a, d, alpha, edge_candidate, point_all, edge, r_cylinder, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone, my_robot);

%%%%%%%%%%%%%%%
%%% 优化参数 %%%
%%%%%%%%%%%%%%%

size_var = 7;

lb = [1,1,1,1,1,1,1];
ub = [size_edge_candidate(1),size_edge_candidate(2),size_edge_candidate(3),size_edge_candidate(4),size_edge_candidate(5),size_edge_candidate(6),size_edge_candidate(7)];

options = optimoptions("ga","Display","iter",'UseParallel',true,'CrossoverFraction',0.7,'CrossoverFcn','crossovertwopoint','MaxGenerations',100,'MigrationFraction',0.01,'FunctionTolerance',1e-2,'PopulationSize',20);

intcon = 1:7;

[solution,objectiveValue] = ga(@(x)detective_rate_multisensor_x(x, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, size_joint, a, d, alpha, edge_candidate, point_all, edge, r_cylinder, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone, my_robot),size_var,[],[],[],[],lb,ub,[],intcon,options);

% idx_helix1 = optimizableVariable('idx_helix1',[1,size_edge_candidate(1)],'Type','integer');
% idx_helix2 = optimizableVariable('idx_helix2',[1,size_edge_candidate(2)],'Type','integer');
% idx_helix3 = optimizableVariable('idx_helix3',[1,size_edge_candidate(3)],'Type','integer');
% idx_helix4 = optimizableVariable('idx_helix4',[1,size_edge_candidate(4)],'Type','integer');
% idx_helix5 = optimizableVariable('idx_helix5',[1,size_edge_candidate(5)],'Type','integer');
% idx_helix6 = optimizableVariable('idx_helix6',[1,size_edge_candidate(6)],'Type','integer');
% idx_helix7 = optimizableVariable('idx_helix7',[1,size_edge_candidate(7)],'Type','integer');
% 
% fun = @(x)detective_rate_multisensor(x.idx_helix1,x.idx_helix2,x.idx_helix3,x.idx_helix4,x.idx_helix5,x.idx_helix6,x.idx_helix7, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, size_joint, a, d, alpha, edge_candidate, point_all, edge, r_cylinder, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone, my_robot);
% results = bayesopt(fun,[idx_helix1,idx_helix2,idx_helix3,idx_helix4,idx_helix5,idx_helix6,idx_helix7],'UseParallel',true);