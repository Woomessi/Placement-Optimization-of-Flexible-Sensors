clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

size_sim = 1; %蒙特卡洛模拟次数

%连杆参数
h_cylinder = 0.09; %圆柱连杆高度
r_cylinder = 0.043; %圆柱连杆半径
size_theta = 10; %圆柱坐标角度均分数

%传感器布局参数
l_sensor = 0.27; %柔性传感器长度
tol_l_sensor = 0.01; %柔性传感器长度容限

%ToF模块参数
size_spot = 16; %ToF模块数量
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
load("my_robot_mesh.mat");

size_joint = 7; %机器人关节数

%机器人关节角约束
joint_constraint = zeros(2,7);
joint_constraint(:,1) = [-166;166];
joint_constraint(:,2) = [-101;101];
joint_constraint(:,3) = [-166;166];
joint_constraint(:,4) = [-176;4];
joint_constraint(:,5) = [-166;166];
joint_constraint(:,6) = [-1;215];
joint_constraint(:,7) = [-166;166];

%利用随机数生成关节空间配置
q = joint_constraint(1,:)*(pi/180) + (joint_constraint(2,:) - joint_constraint(1,:)).*(pi/180).*rand(size_sim,1);

config = homeConfiguration(my_robot); %关节空间配置结构体生成

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

[size_point_target, target] = createObject(0.149, 1.57, 100); %检测目标点生成
target_homo = [target;ones(1,size_point_target)]; %检测目标齐次坐标
translation = [0.5;0;0]; %检测目标平移向量
tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
target_homo = tform_target*target_homo;
target = target_homo(1:3,:); %平移变换后的检测目标

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

for idx_config = 1:size_sim %当前关节角配置
    for idx_joint = 1:size_joint %遍历每个关节
        config(idx_joint).JointPosition = q(idx_config,idx_joint);
    end

    %%%%%%%%%%%%%%%%%%%%%
    %%% 连杆坐标系定义 %%%
    %%%%%%%%%%%%%%%%%%%%%

    %坐标变换
    tform_link = getTransform(my_robot, config, "Joint5_Link", "base_link");%在当前config下，某Joint_Link坐标系到基底坐标系的齐次变换矩阵
    position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

    %%%%%%%%%%%%%%%%%%%%%
    %%% 圆柱螺旋线配置 %%%
    %%%%%%%%%%%%%%%%%%%%%

    idx_helix = 30; %选择当前传感器布局方案
    position_helix = getSpecificHelix(edge_candidate, idx_helix, point_all, edge, r_cylinder, tform_link); %生成特定螺旋线
    [spot, tform_spot_all] = getSpotFrame(size_spot, position_helix, tform_link, position_link, r_cylinder); %生成传感器点相对于世界坐标系的齐次变换矩阵

    %%%%%%%%%%%%%%%%%%%
    %%% ToF模块测距 %%%
    %%%%%%%%%%%%%%%%%%%

    range_all = ones(size_spot,size_point_target); %大型稀疏矩阵，数据结构待优化
    % range_now = ones(size_spot,1);
    % range_old = ones(size_spot,1);
    for idx_point_target = 1:size_point_target
        point_target = target(:,idx_point_target); %目标点设置
        for idx_spot = 1:size_spot
            tform_spot_current = tform_spot_all{1,idx_spot};
            vt = point_target - tform_spot_current(1:3,4); %圆锥顶点到目标点的向量
            l_vt = norm(vt);
            centerline = tform_spot_current(1:3,1); %圆锥中心线
            cos_theta = dot(vt,centerline)/(l_vt*norm(centerline)); %夹角余弦
            if cos_theta > cosd(min(fov_vertical,fov_horizontal)/2) %夹角是否小于视场角的一半？
                if l_vt*cos_theta < h_cone %测距值在中心线方向的投影距离是否在量程内
                    range_all(idx_spot, idx_point_target) = norm(vt);
                    % range_old(idx_spot, 1) = range_now(idx_spot, 1);
                    % if l_vt < range_old(idx_spot, 1)
                    %     range_now(idx_spot, 1) = l_vt;
                    % end
                end
            end
        end
    end
    range = min(range_all,[],2);
end
%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%

show(my_robot,config)
plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot);
scatter3(target(1,:),target(2,:),target(3,:),2,"magenta","filled")
hold off