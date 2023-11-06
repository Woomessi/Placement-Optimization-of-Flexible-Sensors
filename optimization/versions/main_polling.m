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

size_sim = 1000; %蒙特卡洛模拟次数
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
% 作测地线图
% axis equal
% zlim([-h_cylinder 0]);
% grid off;
% hold on
% for i = 1:size_edge_candidate
%     if edge_candidate(i,1) == 0 %共轭测地线满足长度条件
%         idx = edge_candidate(i,2); %离散点对索引
%         %起点圆柱坐标
%         theta1 = point_all(edge(idx ,1),1);
%         z1 = point_all(edge(idx ,1),2);
%         %终点圆柱坐标
%         theta2 = point_all(edge(idx ,2),1);
%         z2 = point_all(edge(idx ,2),2);
%         xi1 = theta1+2*pi;  xi2 = theta2;
%         % 测地线坐标
%         u = linspace(xi1,xi2,50);
%         x = r_cylinder*cos(u);
%         y = r_cylinder*sin(u);
%         z = (z2-z1)/(xi2-xi1)*u + (z1*xi2-z2*xi1)/(xi2-xi1);
%     else %测地线满足长度条件
%         idx = edge_candidate(i,1); %离散点对索引
%         %起点圆柱坐标
%         theta1 = point_all(edge(idx ,1),1);
%         z1 = point_all(edge(idx ,1),2);
%         %终点圆柱坐标
%         theta2 = point_all(edge(idx ,2),1);
%         z2 = point_all(edge(idx ,2),2);
%         % 测地线坐标
%         u = linspace(theta1,theta2,50);
%         x = r_cylinder*cos(u);
%         y = r_cylinder*sin(u);
%         z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
%     end
%     plot3(x,y,z,"LineWidth",1)
% end

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%
% range_each_sim = zeros(size_spot, size_sim);
detective_rate_all = zeros(size_edge_candidate,1);
translation = zeros(3,size_sim);
%利用随机数生成关节空间配置
q_all = zeros(size_sim,size_joint);
for i = 1:size_sim
    q_all(i,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
end
%%%%%%%%%%%%%%%%%%%
%%% 检测目标配置 %%%
%%%%%%%%%%%%%%%%%%%

[size_point_target, target] = createObject(0.149, 1.57, size_theta_object); %检测目标点生成
target_homo = [target;ones(1,size_point_target)]; %检测目标的齐次坐标
% r_obj_offset = 0.27+(0.5-0.27)*rand(size_sim,1);
% theta_obj_offset = 360*rand(size_sim,1);
% for i = 1:size_sim
%     translation(:,i) = [r_obj_offset(i).*cosd(theta_obj_offset(i));r_obj_offset(i).*sind(theta_obj_offset(i));0]; %检测目标平移向量
% end
translation = [0.5;0;0]; %检测目标平移向量
tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
% tform_target = [eye(3),translation(:,idx_config);0 0 0 1]; %平移变换矩阵
target_homo = tform_target*target_homo;
target = target_homo(1:3,:); %平移变换后的检测目标
parfor idx_helix = 1:size_edge_candidate
    detection_times = 0; %检测到目标的次数
    for idx_config = 1:size_sim %当前关节角配置

        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        q = q_all(idx_config,:);
        % tform_target = [eye(3),translation(:,idx_config);0 0 0 1]; %平移变换矩阵
        % target_homo = tform_target*target_homo;
        % target = target_homo(1:3,:); %平移变换后的检测目标
        %坐标变换
        tform_link = eye(4);
        for i = 1:idx_link
            tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
        end
        position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

        %%%%%%%%%%%%%%%%%%%%%
        %%% 圆柱螺旋线配置 %%%
        %%%%%%%%%%%%%%%%%%%%%

        position_helix = getSpecificHelix(edge_candidate, idx_helix, point_all, edge, r_cylinder, tform_link); %生成特定螺旋线
        [spot, tform_spot_all] = getSpotFrame(size_spot, position_helix, tform_link, position_link, r_cylinder); %生成传感器点相对于世界坐标系的齐次变换矩阵

        %%%%%%%%%%%%%%%%%%%
        %%% ToF模块测距 %%%
        %%%%%%%%%%%%%%%%%%%

        % detection_times = getRange(size_point_target, target, size_spot, tform_spot_all, fov_vertical, fov_horizontal, h_cone, detection_times);
        % range_all = ones(size_spot,size_point_target); %大型稀疏矩阵，数据结构待优化
        % range_now = ones(size_spot,1);
        % range_old = ones(size_spot,1);

        for idx_point_target = 1:size_point_target
            flag = 0; %循环跳出标识
            point_target = target(:,idx_point_target); %目标点设置
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

        % range = min(range_all,[],2);
        % range_each_sim(:,idx_config) = min(range_all,[],2);
    end
    detective_rate = detection_times/size_sim;
    detective_rate_all(idx_helix) = detective_rate;
end
%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%

% config = homeConfiguration(my_robot); %关节空间配置结构体生成
% for i = 1:size_joint %遍历每个关节
%     config(i).JointPosition = q(idx_config,i);
% end
% show(my_robot,config)
% plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot);
% scatter3(target(1,:),target(2,:),target(3,:),2,"magenta","filled")
% hold off