clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%
%%% 优化变量 %%%
%%%%%%%%%%%%%%%

size_spot = 16; % ToF模块数量

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

%{
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 提取与传感器长度相近的测地线 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
edge_candidate_all = cell(1,size_joint);
for idx_link = 1:size_joint % 遍历所有连杆
% for idx_link = 1 % 遍历所有连杆
    edge_candidate = [];
    X = X_link{1,idx_link}(1:3,:);
    size_X = size(X,2);
    F = F_link{1,idx_link}';
    for idx_point = 1:size_X
        %%%%%%%%%%%%%%%%%%%
        %%% Heat Method %%%
        %%%%%%%%%%%%%%%%%%%

        % pstart = idx_point; % 起点序号
        % D = getGeodesicDistanceHeat(X,F,pstart);
        % D(1:idx_point) = 0; % 删除重复终点
        % edge_candidate1 = find(abs(D-l_sensor)<tol_l_sensor); %储存满足长度条件的待选离散点对的数组
        % edge_candidate0 = [idx_point*ones(size(edge_candidate1,1),1),edge_candidate1];
        % edge_candidate = [edge_candidate;edge_candidate0];

        %%%%%%%%%%%%%%%%%%%%%
        %%% Fast Marching %%%
        %%%%%%%%%%%%%%%%%%%%%

        pstart = idx_point;
        D = perform_fast_marching_mesh(X, F, pstart);
        % D(1:idx_point) = 0; % 删除重复终点
        edge_candidate1 = find(abs(D-l_sensor)<tol_l_sensor); %储存满足长度条件的待选离散点对的数组
        edge_candidate0 = [idx_point*ones(size(edge_candidate1,1),1),edge_candidate1];
        edge_candidate = [edge_candidate;edge_candidate0];
    end
    edge_candidate_all{1,idx_link} = edge_candidate;
end
for i = 1:size_joint
edge_candidate_all{1,i} = unique(sort(edge_candidate_all{1,i},2),"rows");
end
%}

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
% idx_helix = [4984 390 6431 14 12186 3861 56];

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

detection_times = 0; % 检测到目标的次数
for idx_config = 1:size_sim

    %%%%%%%%%%%%%%%%%%%%%%
    %%% 检测目标坐标信息 %%%
    %%%%%%%%%%%%%%%%%%%%%%

    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; % 检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; % 平移变换矩阵
    target_homo1 = tform_target*target_homo;
    target1 = target_homo1(1:3,:); % 平移变换后的检测目标

    %%%%%%%%%%%%%%%%%%%%%
    %%% 当前关节角配置 %%%
    %%%%%%%%%%%%%%%%%%%%%

    q = q_all(idx_config,:); % 当前关节空间角
    link_name = ["panda_link0" "panda_link1" "panda_link2" "panda_link3" "panda_link4" "panda_link5" "panda_link6" "panda_link7"];
    config = homeConfiguration(my_robot); % 关节空间配置结构体生成
    for i = 1:size_joint % 遍历每个关节
        config(i).JointPosition = q(1,i);
    end

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% 各连杆传感器配置 %%%
    %%%%%%%%%%%%%%%%%%%%%%%

    for idx_link = 1:size_joint % 遍历所有连杆 (末端连杆对应空矩阵)
        % for idx_link = size_joint
        transform = getTransform(my_robot,config,link_name(idx_link)); %当前位形空间下连杆相对于基坐标系的齐次变换矩阵
        X_update = transform*X_link{1,idx_link}; % 当前时刻连杆的mesh坐标
        F = (double(F_link{1,idx_link}))'; % 当前连杆mesh的三角面片信息
        size_F = size(F,2); % 三角面数量
        X = (double(X_update(1:3,:))); % 当前连杆mesh的顶点坐标信息
        size_X = size(X,2); % 三角面片顶点数量

        TR = triangulation(F',X'); % MATLAB官方的三角面片数据结构

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 检查连杆三角面片模型 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        % figure(1);
        % trimesh(TR);
        % view(135,7);
        % axis equal
        % hold on
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 连杆mesh模型几何信息提取 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        N_f = faceNormal(TR); % 计算三角面法向量
        N_v = vertexNormal(TR); % 计算顶点法向量

        C = incenter(TR); % 计算三角面形心
        X_refined = [X,C']; % 向顶点坐标矩阵中加入三角面形心坐标
        N_refined = [N_v;N_f]; % 向顶点法向量矩阵中加入三角面法向量，作为三角面片形心法向量

        % 不引入形心
        % X_refined = X; % 向顶点坐标矩阵中加入三角面形心坐标
        % N_refined = N_v; % 向顶点法向量矩阵中加入三角面法向量，作为三角面片形心法向量

        % quiver3(C(:,1),C(:,2),C(:,3), ...
        %        N_f(:,1), N_f(:,2), N_f(:,3),5,'Color','C');
        %
        % quiver3(X(1,:),X(2,:),X(3,:), ...
        %        N_v(:,1)', N_v(:,2)', N_v(:,3)',5,'Color','M');

        %%%%%%%%%%%%%%%%%%%
        %%% Heat Method %%%
        %%%%%%%%%%%%%%%%%%%

        pstart = edge_candidate_all{1,idx_link}(idx_helix(idx_link),1); % 起点序号
        phi = getGeodesicDistanceHeat(X,F,pstart);

        %%%%%%%%%%%%%%%%%%%%%
        %%% 计算测地线路径 %%%
        %%%%%%%%%%%%%%%%%%%%%

        pend = edge_candidate_all{1,idx_link}(idx_helix(idx_link),2); % 终点序号
        %
        % options.method = 'continuous';
        % options.method = 'discrete';
        %
        % options.face_vertex_color = phi;
        paths = compute_geodesic_mesh(phi, X, F, pend);
        paths = unique(paths',"rows","stable");
        paths = paths';

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%
        
        % plot_fast_marching_mesh(X, F, phi, paths);

        % [phi,paths] = heatMethod(X,F,pstart,pend); % 用Heat Method求测地线距离与路径

        %%%%%%%%%%%%%%%%%%%%%
        %%% 路径拟合与提取 %%%
        %%%%%%%%%%%%%%%%%%%%%

        % scatter3(paths(1,:),paths(2,:),paths(3,:),18,'magenta','filled') % 绘制路径点

        size_interval = 200; % 拟合路径点间距数
        interval1 = (max(paths(1,:))-min(paths(1,:)))/size_interval; % 拟合路径点间距
        interval2 = (max(paths(2,:))-min(paths(2,:)))/size_interval; % 拟合路径点间距
        interval3 = (max(paths(3,:))-min(paths(3,:)))/size_interval; % 拟合路径点间距

        % 选择间距最大的方向作为插值方向，提高插值效果
        pp = [min(paths(1,:)):interval1:max(paths(1,:));min(paths(2,:)):interval2:max(paths(2,:));min(paths(3,:)):interval3:max(paths(3,:))];
        [~,idx_pp] = max([max(paths(1,:))-min(paths(1,:)),max(paths(2,:))-min(paths(2,:)),max(paths(3,:))-min(paths(3,:))]);

        coefficient_x = polyfit(paths(idx_pp,:),paths(1,:),5); % y-x三次多项式曲线拟合系数
        x = polyval(coefficient_x,pp(idx_pp,:)); % 拟合路径点的y坐标
        coefficient_y = polyfit(paths(idx_pp,:),paths(2,:),5); % y-x三次多项式曲线拟合系数
        y = polyval(coefficient_y,pp(idx_pp,:)); % 拟合路径点的y坐标
        coefficient_z = polyfit(paths(idx_pp,:),paths(3,:),5); % z-x三次多项式曲线拟合系数
        z = polyval(coefficient_z,pp(idx_pp,:)); % 拟合路径点的z坐标

        % x = makima(paths(idx_pp,:),paths(1,:),pp(idx_pp,:));
        % y = makima(paths(idx_pp,:),paths(2,:),pp(idx_pp,:));
        % z = makima(paths(idx_pp,:),paths(3,:),pp(idx_pp,:));

        % x = spline(paths(idx_pp,:),paths(1,:),pp(idx_pp,:));
        % y = spline(paths(idx_pp,:),paths(2,:),pp(idx_pp,:));
        % z = spline(paths(idx_pp,:),paths(3,:),pp(idx_pp,:));

        % point_path = [x;y;z]; % 拟合路径点
        point_path = paths; % 不使用拟合路径点

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 剔除在当前连杆范围之外的路径点 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        x_max = max(X(1,:));
        y_max = max(X(2,:));
        z_max = max(X(3,:));

        x_min = min(X(1,:));
        y_min = min(X(2,:));
        z_min = min(X(3,:));

        for i = 1:size(point_path,2)
            if point_path(1,i) > x_max || point_path(1,i) < x_min
                point_path(:,i) = zeros(3,1);
            else
                if point_path(2,i) > y_max || point_path(2,i) < y_min
                    point_path(:,i) = zeros(3,1);
                else
                    if point_path(3,i) > z_max || point_path(3,i) < z_min
                        point_path(:,i) = zeros(3,1);
                    end
                end
            end
        end
        point_path(:,all(point_path==0,1))=[]; % 剔除全0列
        size_point_path = size(point_path,2); % 最终的拟合路径点数量

        % scatter3(point_path(1,:),point_path(2,:),point_path(3,:),18,'magenta','filled') % 绘制路径点
        % plot3(point_path(1,:),point_path(2,:),point_path(3,:),'LineWidth',1,'Color','m') % 绘制拟合路径

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 拟合路径点坐标系建立 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        % 在三角面片顶点与形心点中提取距各拟合路径点最近的点
        ID_all = dsearchn(X_refined',point_path');

        % 将各最近点的法向量作为相应路径点的法向量 (x轴基向量)
        x_point_path = N_refined(ID_all,:)';

        % 初步选定相邻路径点之间的连线为坐标系的y轴基向量
        y_point_path = zeros(3,size_point_path);
        for i = 1:size_point_path-1
            y_point_path(:,i) = point_path(:,i+1) - point_path(:,i);
            y_point_path(:,i) = y_point_path(:,i)./norm(y_point_path(:,i));
        end
        y_point_path(:,size_point_path) = y_point_path(:,size_point_path-1);

        % 通过叉乘获得z轴基向量
        z_point_path = cross(x_point_path,y_point_path);
        z_point_path = z_point_path./vecnorm(z_point_path);

        % 通过叉乘修正y轴基向量
        y_point_path = cross(z_point_path,x_point_path);

        % 绘制拟合路径点坐标系
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     x_point_path(1,:),x_point_path(2,:),x_point_path(3,:),0.5,'Color','r');
        % 
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     y_point_path(1,:),y_point_path(2,:),y_point_path(3,:),0.5,'Color','g');
        % 
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     z_point_path(1,:),z_point_path(2,:),z_point_path(3,:),0.5,'Color','b');

        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        %坐标变换
        tform_link = eye(4);
        for i = 1:idx_link
            tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
        end
        % position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

        % 在拟合路径点中提取均分点，作为TOF模块点，并计算相应齐次变换矩阵
        tform_spot_all = getUniformPointFrames(size_spot, point_path, size_point_path, x_point_path, y_point_path, z_point_path);

        %%%%%%%%%%%%%%%
        %%% ToF检测 %%%
        %%%%%%%%%%%%%%%

        for idx_point_target = 1:size_point_target
            flag = 0; %循环跳出标识
            point_target = target1(:,idx_point_target); %目标点设置
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot_all{1,idx_spot};
                if tform_spot_current(:,1) == zeros(4,1)
                    break;
                end
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
        if flag == 1 %跳出三重循环
            break
        end
        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%
        show(my_robot,config); % 打开figure属性
        hold on
        plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot)
        scatter3(target1(1,:),target1(2,:),target1(3,:),2,"magenta","filled")
        view(0,90)
        hold off
    end
end
detective_rate = detection_times/size_sim;