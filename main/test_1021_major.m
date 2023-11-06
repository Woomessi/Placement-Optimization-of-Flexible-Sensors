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

%%%%%%%%%%%%%%%%%%%%%
%%% 所有关节角配置 %%%
%%%%%%%%%%%%%%%%%%%%%

% 利用随机数生成关节空间位形
q_all = zeros(size_sim,size_joint);
for i = 1:size_sim
    q_all(i,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
end

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

for idx_config = 1:1

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

    for idx_link = 7 % 遍历所有连杆 (idx_link = 1 对应 link0，末端连杆对应空矩阵)
        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        % 坐标变换
        transform = getTransform(my_robot,config,link_name(idx_link)); %当前位形空间下连杆相对于基坐标系的齐次变换矩阵

        % transform = eye(4);
        % if idx_link == 1
        %     transform = eye(4);
        % else
        %     for i = 1:idx_link-1
        %         transform = transform*getTformMDH(a(i),d(i),alpha(i),q(i));
        %     end
        % end

        % 在拟合路径点中提取均分点，作为TOF模块点，并计算相应齐次变换矩阵
        load("all_tform_spot_link6.mat");
        tform_spot = all_tform_spot{1,1};
        for i = 1:size_spot
            tform_spot{1,i} = transform*tform_spot{1,i};
        end

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%
        show(my_robot,config); % 打开figure属性
        hold on
        for i = 1:size_spot
            scatter3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4),18,"magenta",'filled')
            % 绘制拟合路径点坐标系
            quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
                tform_spot{1,i}(1,1),tform_spot{1,i}(2,1),tform_spot{1,i}(3,1),0.05,'Color','r');

            quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
                tform_spot{1,i}(1,2),tform_spot{1,i}(2,2),tform_spot{1,i}(3,2),0.05,'Color','g');

            quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
                tform_spot{1,i}(1,3),tform_spot{1,i}(2,3),tform_spot{1,i}(3,3),0.05,'Color','b');
        end

        plotFoV(h_cone, fov_horizontal, tform_spot, size_spot)
        view(0,90)
        hold off
    end
end