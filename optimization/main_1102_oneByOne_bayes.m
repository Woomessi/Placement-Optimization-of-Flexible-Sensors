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
size_theta_object = 50; % 检测目标圆柱坐标角度均分数

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

idx_link = 0:6;
idx_geodesic = [6929, 1142, 20761, 10200, 10451, 13100, 17];% 严格按顺序

% idx_link = [0,1,2,4,5,6];
% idx_geodesic = [6929, 1142, 20761, 10451, 13100, 17];% 严格按顺序

detective_rate = -getDetectiveRate_multiple_specific(idx_link, idx_geodesic, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% detective_rate = getDetectiveRate_single(1, 11529, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% detective_rate = getDetectiveRate_multiple([1,2], [529,1629], 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% detective_rate = getDetectiveRate_multiple3(0, 5, 100, 200, 300, 400, 500, 600, 700,  16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%
%%% 优化参数 %%%
%%%%%%%%%%%%%%%

idx_geodesic0 = optimizableVariable('idx_geodesic0',[1,size_all_tform_spot_link0],'Type','integer');
idx_geodesic1 = optimizableVariable('idx_geodesic1',[1,size_all_tform_spot_link1],'Type','integer');
idx_geodesic2 = optimizableVariable('idx_geodesic2',[1,size_all_tform_spot_link2],'Type','integer');
idx_geodesic3 = optimizableVariable('idx_geodesic3',[1,size_all_tform_spot_link3],'Type','integer');
idx_geodesic4 = optimizableVariable('idx_geodesic4',[1,size_all_tform_spot_link4],'Type','integer');
idx_geodesic5 = optimizableVariable('idx_geodesic5',[1,size_all_tform_spot_link5],'Type','integer');
idx_geodesic6 = optimizableVariable('idx_geodesic6',[1,size_all_tform_spot_link6],'Type','integer');

idx_link1 = optimizableVariable('idx_link1',[0,6],'Type','integer');
idx_link2 = optimizableVariable('idx_link2',[0,5],'Type','integer');
idx_link3 = optimizableVariable('idx_link3',[1,5],'Type','integer');
idx_link4 = optimizableVariable('idx_link4',[1,4],'Type','integer');
idx_link5 = optimizableVariable('idx_link5',[2,4],'Type','integer');
idx_link6 = optimizableVariable('idx_link6',[3,4],'Type','integer');
idx_link7 = optimizableVariable('idx_link7',[0,6],'Type','integer');

% idx1 = optimizableVariable('idx1',[0,6],'Type','integer');
% idx2 = optimizableVariable('idx2',[0,6],'Type','integer');
% idx3 = optimizableVariable('idx3',[0,6],'Type','integer');
% idx4 = optimizableVariable('idx4',[0,6],'Type','integer');
% idx5 = optimizableVariable('idx5',[0,6],'Type','integer');
% idx6 = optimizableVariable('idx6',[0,6],'Type','integer');
% idx7 = optimizableVariable('idx7',[0,6],'Type','integer');

% flag_sensor0 = optimizableVariable('flag_sensor0',[0,1],'Type','integer');
% flag_sensor1 = optimizableVariable('flag_sensor1',[0,1],'Type','integer');
% flag_sensor2 = optimizableVariable('flag_sensor2',[0,1],'Type','integer');
% flag_sensor3 = optimizableVariable('flag_sensor3',[0,1],'Type','integer');
% flag_sensor4 = optimizableVariable('flag_sensor4',[0,1],'Type','integer');
% flag_sensor5 = optimizableVariable('flag_sensor5',[0,1],'Type','integer');
% flag_sensor6 = optimizableVariable('flag_sensor6',[0,1],'Type','integer');

% 指定spot的数量
size_spot = 16;
% 优化spot的数量
% size_spot = optimizableVariable('size_spot',[1,16],'Type','integer');

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用多个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

% size_sensor = 4;
% fun = @(x)getDetectiveRate_multiple(size_sensor, x.idx1, x.idx2, x.idx3, x.idx4, x.idx5, x.idx6, x.idx7, x.idx_geodesic0, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, x.idx_geodesic6,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx1, idx2, idx3, idx4, idx5, idx6, idx7, idx_geodesic0, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5, idx_geodesic6],'UseParallel',true,'PlotFcn',{@plotAcquisitionFunction,@plotObjectiveModel,@plotObjective,@plotMinObjective});

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 只使用1个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

fun = @(x)getDetectiveRate_1(x.idx_link1, x.idx_geodesic0, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, x.idx_geodesic6,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
results = bayesopt(fun,[idx_link1, idx_geodesic0, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5, idx_geodesic6],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用2个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_double(6, x.idx_link2, x.idx_geodesic0, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link2, idx_geodesic0, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用3个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_3(6, 0, x.idx_link3, 6929, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link3, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用4个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_4(6, 0, 5, x.idx_link4, 6929, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link4, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用5个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_5(6, 0, 5, 1, x.idx_link5, 6929, 1142, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link5, idx_geodesic2, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用6个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_6(6, 0, 5, 1, 2, x.idx_link6, 6929, 1142, 20761, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link6, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用7个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_7(6, 0, 5, 1, 2, 4, 3, 6929, 1142, 20761, x.idx_geodesic3, 10451, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,idx_geodesic3,'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用多个柔性传感器，并指定传感器所在关节 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = [1,2];
% fun = @(x)getDetectiveRate_multiple_specific(idx_link, [x.idx_geodesic1,x.idx_geodesic2], size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_geodesic1,idx_geodesic2],'UseParallel',true,'PlotFcn',[]);

toc