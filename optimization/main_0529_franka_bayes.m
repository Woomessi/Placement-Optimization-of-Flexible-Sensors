clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

% 机器人连杆参数
size_sim = 3000; % 蒙特卡洛模拟次数
size_theta_object = 25; % 检测目标圆柱坐标角度均分数

% 传感器布局参数
l_sensor = 0.27; % 柔性传感器长度
tol_l_sensor = 0.001; % 柔性传感器长度容限

% ToF模块参数
fov_horizontal = 25; % 水平视场角，单位为degree
fov_vertical = 25; % 垂直视场角，单位为degree
range_max = 0.5; % 最大检测距离
h_cone = range_max*cosd(fov_horizontal/2); % 视场锥高度

%%%%%%%%%%%%%%%%%
%%% 机器人定义 %%%
%%%%%%%%%%%%%%%%%

% 连杆三角面片模型输入
TR0 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link0.stl');
X0 = TR0.Points';
X0 = [X0;ones(1,size(X0,2))];
F0 = TR0.ConnectivityList;

TR1 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link1.stl');
X1 = TR1.Points';
X1 = [X1;ones(1,size(X1,2))];
F1 = TR1.ConnectivityList;

TR2 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link2.stl');
X2 = TR2.Points';
X2 = [X2;ones(1,size(X2,2))];
F2 = TR2.ConnectivityList;

TR3 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link3.stl');
X3 = TR3.Points';
X3 = [X3;ones(1,size(X3,2))];
F3 = TR3.ConnectivityList;

TR4 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link4.stl');
X4 = TR4.Points';
X4 = [X4;ones(1,size(X4,2))];
F4 = TR4.ConnectivityList;

TR5 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link5.stl');
X5 = TR5.Points';
X5 = [X5;ones(1,size(X5,2))];
F5 = TR5.ConnectivityList;

TR6 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link6.stl');
X6 = TR6.Points';
X6 = [X6;ones(1,size(X6,2))];
F6 = TR6.ConnectivityList;

TR7 = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link7.stl');
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

%%%%%%%%%%%%%%%%%%%
%%% 检测目标配置 %%%
%%%%%%%%%%%%%%%%%%%

% createObject参数：半径、高度（取中国女性平均身高）、圆柱坐标均分数
[size_point_target, target] = createObject(0.149, 1.57, size_theta_object); % 检测目标点生成
target_homo = [target;ones(1,size_point_target)]; % 检测目标的齐次坐标
r_obj_offset = 0.27+(0.5-0.27)*rand(size_sim,1); % 相对于机器人的距离随机偏移量
theta_obj_offset = 360*rand(size_sim,1); % 相对于机器人的角度随机偏移量

%%%%%%%%%%%%%%%%%%%%%
%%% 所有关节角配置 %%%
%%%%%%%%%%%%%%%%%%%%%

% 利用随机数生成关节空间位形
q_all = zeros(size_sim,size_joint);
for i = 1:size_sim
    q_all(i,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 提取与传感器长度相近的测地线 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load("edge_candidate_all2.mat",'edge_candidate_all1');
load("edge_candidate7.mat",'edge_candidate');
edge_candidate_all1{1,7} = edge_candidate;

size_edge_candidate = zeros(1,size_joint);
for i = 1:size_joint
size_edge_candidate(i) = size(edge_candidate_all1{1,i},1);
end
% idx_helix = [randi([1,size_edge_candidate(1)]),randi([1,size_edge_candidate(2)]),randi([1,size_edge_candidate(3)]),randi([1,size_edge_candidate(4)]),randi([1,size_edge_candidate(5)]),randi([1,size_edge_candidate(6)]),randi([1,size_edge_candidate(7)])]; % 选择当前传感器布局方案

%%%%%%%%%%%%%%%
%%% 优化参数 %%%
%%%%%%%%%%%%%%%

idx_helix1 = optimizableVariable('idx_helix1',[1,size_edge_candidate(1)],'Type','integer');
idx_helix2 = optimizableVariable('idx_helix2',[1,size_edge_candidate(2)],'Type','integer');
idx_helix3 = optimizableVariable('idx_helix3',[1,size_edge_candidate(3)],'Type','integer');
idx_helix4 = optimizableVariable('idx_helix4',[1,size_edge_candidate(4)],'Type','integer');
idx_helix5 = optimizableVariable('idx_helix5',[1,size_edge_candidate(5)],'Type','integer');
idx_helix6 = optimizableVariable('idx_helix6',[1,size_edge_candidate(6)],'Type','integer');
idx_helix7 = optimizableVariable('idx_helix7',[1,size_edge_candidate(7)],'Type','integer');

% ring1 = optimizableVariable('ring1',[0,1],'Type','integer');
% ring2 = optimizableVariable('ring2',[0,1],'Type','integer');
% ring3 = optimizableVariable('ring3',[0,1],'Type','integer');
% ring4 = optimizableVariable('ring4',[0,1],'Type','integer');
% ring5 = optimizableVariable('ring5',[0,1],'Type','integer');
% ring6 = optimizableVariable('ring6',[0,1],'Type','integer');
% ring7 = optimizableVariable('ring7',[0,1],'Type','integer');

% 指定spot的数量
size_spot = 16;
% 优化spot的数量
% size_spot = optimizableVariable('size_spot',[1,16],'Type','integer');

% 使用所有ring
fun = @(x)detective_rate_franka(x.idx_helix1,x.idx_helix2,x.idx_helix3,x.idx_helix4,x.idx_helix5,x.idx_helix6,x.idx_helix7,size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, X_link, F_link, edge_candidate_all1, a, d, alpha, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone);
results = bayesopt(fun,[idx_helix1,idx_helix2,idx_helix3,idx_helix4,idx_helix5,idx_helix6,idx_helix7],'UseParallel',true,'GPActiveSetSize',300,'ExplorationRatio',0.5,'ParallelMethod','clipped-model-prediction','MaxObjectiveEvaluations',30,'NumSeedPoints',4);

% 优化ring的数量
% fun = @(x)detective_rate_franka_varyingsensor(x.idx_helix1,x.idx_helix2,x.idx_helix3,x.idx_helix4,x.idx_helix5,x.idx_helix6,x.idx_helix7,x.ring1,x.ring2,x.ring3,x.ring4,x.ring5,x.ring6,x.ring7,size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, X_link, F_link, edge_candidate_all1, a, d, alpha, x.size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_helix1,idx_helix2,idx_helix3,idx_helix4,idx_helix5,idx_helix6,idx_helix7,ring1,ring2,ring3,ring4,ring5,ring6,ring7,size_spot],'UseParallel',true,'GPActiveSetSize',300,'ExplorationRatio',0.5,'ParallelMethod','clipped-model-prediction','MaxObjectiveEvaluations',30,'NumSeedPoints',4);
