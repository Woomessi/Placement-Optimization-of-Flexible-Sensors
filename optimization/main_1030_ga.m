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

% detective_rate = getDetectiveRate_single(1, 11529, 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% detective_rate = getDetectiveRate_multiple([1,2], [529,1629], 16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% detective_rate = getDetectiveRate_multiple3(0, 5, 100, 200, 300, 400, 500, 600, 700,  16, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);

%%%%%%%%%%%%%%%
%%% 优化参数 %%%
%%%%%%%%%%%%%%%

% 指定spot的数量
size_spot = 16;
% 优化spot的数量
% size_spot = optimizableVariable('size_spot',[1,16],'Type','integer');

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用多个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

size_sensor = 4;
lb = [0,0,0,0,0,0,0,1,1,1,1,1,1,1];
ub = [6,6,6,6,6,6,6,size_all_tform_spot_link0,size_all_tform_spot_link1,size_all_tform_spot_link2,size_all_tform_spot_link3,size_all_tform_spot_link4,size_all_tform_spot_link5,size_all_tform_spot_link6];% 待修改
size_var = 14;
intcon = 1:size_var;
initialSpan = 1000;
% options = optimoptions("ga","Display","iter",'UseParallel',true);
options = optimoptions("ga","Display","iter",'UseParallel',true,'CrossoverFraction',0.7,'CrossoverFcn','crossovertwopoint','MaxGenerations',100,'MigrationFraction',0.01,'FunctionTolerance',1e-2,'PopulationSize',20,'MaxTime',9000);
fun = @(x)getDetectiveRate_multiple_GA(size_sensor, x, size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
[solution,objectiveValue] = ga(fun,size_var,[],[],[],[],lb,ub,[],intcon,options);

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 使用2个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%

% lb = [0,0,1,1,1,1,1,1,1];
% ub = [6,6,size_all_tform_spot_link0,size_all_tform_spot_link1,size_all_tform_spot_link2,size_all_tform_spot_link3,size_all_tform_spot_link4,size_all_tform_spot_link5,size_all_tform_spot_link6];% 待修改
% size_var = 9;
% intcon = 1:9;
% initialSpan = 1000;
% % options = optimoptions("ga","Display","iter",'UseParallel',true);
% options = optimoptions("ga","Display","iter",'UseParallel',true,'CrossoverFraction',0.7,'CrossoverFcn','crossovertwopoint','MaxGenerations',100,'MigrationFraction',0.01,'FunctionTolerance',1e-2,'PopulationSize',20,'MaxTime',9000);
% fun = @(x)getDetectiveRate_double_GA(x, size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% [solution,objectiveValue] = ga(fun,size_var,[],[],[],[],lb,ub,[],intcon,options);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 只使用1个柔性传感器 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

% idx_link = 2;% 待修改
% ub = size_all_tform_spot_link2;% 待修改
% lb = 1;
% size_var = 1;
% intcon = 1;
% initialSpan = 1000;
% % options = optimoptions("ga","Display","iter",'UseParallel',true);
% options = optimoptions("ga","Display","iter",'UseParallel',true,'CrossoverFraction',0.7,'CrossoverFcn','crossovertwopoint','MaxGenerations',100,'MigrationFraction',0.01,'FunctionTolerance',1e-2,'PopulationSize',20,'MaxTime',9000);
% fun = @(x)getDetectiveRate_single(idx_link, x, size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% [solution,objectiveValue] = ga(fun,size_var,[],[],[],[],lb,ub,[],intcon,options);

toc