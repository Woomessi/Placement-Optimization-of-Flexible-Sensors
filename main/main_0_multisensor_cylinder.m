clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%
%%% 优化变量 %%%
%%%%%%%%%%%%%%%

size_spot = 16; %ToF模块数量

% idx_helix = [2893,2634,2192,1578,1615,2766,17]; %选择当前传感器布局方案

idx_helix = [300,300,300,300,300,300,300]; %选择当前传感器布局方案

%%%%%%%%%%%%%%%%%%%
%%% 基本参数设置 %%%
%%%%%%%%%%%%%%%%%%%

%机器人连杆参数
h_cylinder = [0.09,0.09,0.09,0.09,0.09,0.09,0.09]; %圆柱连杆高度
r_cylinder = [0.043,0.043,0.043,0.043,0.043,0.043,0.043]; %圆柱连杆半径

size_sim = 3000; %蒙特卡洛模拟次数
% size_theta = 50; %连杆圆柱坐标角度均分数
size_theta = 25; %连杆圆柱坐标角度均分数
% size_theta_object = 25; %目标圆柱坐标角度均分数
size_theta_object = 15; %目标圆柱坐标角度均分数

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

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

detection_times = 0; %检测到目标的次数
for idx_config = 1:size_sim %当前关节角配置

    %%%%%%%%%%%%%%%%
    %%% 检测目标 %%%
    %%%%%%%%%%%%%%%%

    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; %检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
    target_homo1 = tform_target*target_homo;
    target1 = target_homo1(1:3,:); %平移变换后的检测目标

    q = q_all(idx_config,:);

    for idx_link = 1:size_joint

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%

        % plotHelix1(h_cylinder(idx_link), size_edge_candidate, edge_candidate, point_all, edge, r_cylinder(idx_link));

        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        %坐标变换
        tform_link = eye(4);
        for i = 1:idx_link
            tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
        end
        position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

        %%%%%%%%%%%%%%%%%%%%%
        %%% 圆柱螺旋线配置 %%%
        %%%%%%%%%%%%%%%%%%%%%

        position_helix = getSpecificHelix(edge_candidate{1,idx_link}, idx_helix(idx_link), point_all{1,idx_link}, edge{1,idx_link}, r_cylinder(idx_link), tform_link); %生成特定螺旋线
        [spot, tform_spot_all] = getSpotFrame(size_spot, position_helix, tform_link, position_link, r_cylinder(idx_link)); %生成传感器点相对于世界坐标系的齐次变换矩阵

        for idx_point_target = 1:size_point_target
            flag = 0; %循环跳出标识
            point_target = target1(:,idx_point_target); %目标点设置
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot_all{1,idx_spot};
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
                        flag = 1;
                        detection_times = detection_times + 1;
                        break
                    end
                end
            end
            if flag == 1 %跳出两重循环
                break
            end
        end

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%

        config = homeConfiguration(my_robot); %关节空间配置结构体生成
        for i = 1:size_joint %遍历每个关节
            config(i).JointPosition = q(1,i);
        end
        show(my_robot,config)
        plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot)
        % scatter3(spot(1,:),spot(2,:),spot(3,:),2,"magenta","filled")
        scatter3(target1(1,:),target1(2,:),target1(3,:),2,"magenta","filled")
        view(0,90)
        hold off
    end
end
detective_rate = detection_times/(size_sim*size_joint);