clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

% 圆柱螺旋线设置
r_cylinder = 0.043; %圆柱连杆半径
l_sensor = 0.27; %柔性传感器环长度
h_cylinder = 0.09; %圆柱连杆高度
size_sim = 1; %蒙特卡洛法次数
size_sensor = 16;

%%%%%%%%%%%%%%%%%%%
%%% 离散化圆柱面 %%%
%%%%%%%%%%%%%%%%%%%
size_theta = 10; %圆柱坐标角度均分数
[size_point, point_all] = discretizeCylinder(r_cylinder, h_cylinder, size_theta);

%%%%%%%%%%%%%%%%%%%%%
%%% 测地线生成 %%%
%%%%%%%%%%%%%%%%%%%%%
[edge, size_edge, l_edge] = getHelix(size_point, point_all, r_cylinder);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 提取与传感器长度相近的测地线 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tol = 0.01; %长度容限
[edge_candidate, size_edge_candidate] = pickHelix(tol, size_edge, l_edge, l_sensor);

%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%
% plotHelix(r_cylinder, h_cylinder, size_edge_candidate, edge_candidate, point_all, edge);

%%%%%%%%%%%%%%%%%%%%%%%
%%% 初始化机器人仿真 %%%
%%%%%%%%%%%%%%%%%%%%%%%

%初始化仿真环境
sampleRate = 100; %与传感器更新频率一致
scenario = robotScenario(UpdateRate=sampleRate, MaxNumFrames=100); %机器人仿真需要较高的总帧数

%机器人生成
my_robot = importrobot('model_sensor_compact.urdf'); %无mesh，运行更快
% my_robot = importrobot('model_sensor_compact_win.urdf');

robot = robotPlatform("rst", scenario, RigidBodyTree=my_robot);

%机器人关节角约束

joint_constraint = zeros(2,7);
joint_constraint(:,1) = [-166;166];
joint_constraint(:,2) = [-101;101];
joint_constraint(:,3) = [-166;166];
joint_constraint(:,4) = [-176;4];
joint_constraint(:,5) = [-166;166];
joint_constraint(:,6) = [-1;215];
joint_constraint(:,7) = [-166;166];

q = joint_constraint(1,:)*(pi/180) + (joint_constraint(2,:) - joint_constraint(1,:)).*(pi/180).*rand(size_sim,1);

config = homeConfiguration(my_robot); %关节空间配置结构体生成
for idx_config = 1:size_sim %遍历每组关节空间配置
    % robot = robotPlatform("rst"+num2str(idx_config), scenario, RigidBodyTree=my_robot);
    % idx_config = 2; %当前关节角配置
    for idx_joint = 1:7 %遍历每个关节
        config(idx_joint).JointPosition = q(idx_config,idx_joint);
    end

    %坐标变换
    tform = getTransform(my_robot, config, "Joint5_Link", "base_link");%在当前config下，某Joint_Link坐标系到基底坐标系的齐次变换矩阵
    position = tform(1:3,4)';%Joint_Link坐标系原点在基底坐标系中的坐标
    rpy_zyx = tform2eul(tform);%Joint_Link坐标系姿态相对于基底坐标系的rpy角表示

    %%%%%%%%%%%%%%%%%%%%%
    %%% 圆柱螺旋线配置 %%%
    %%%%%%%%%%%%%%%%%%%%%

    idx_geodesic = 30; %选择当前传感器布局方案
    position_helix = getSpecificHelix(edge_candidate, idx_geodesic, point_all, edge, r_cylinder, tform); %生成特定螺旋线
    [spot, tform_sensor_all] = getSpotFrame(size_sensor, position_helix, tform, position, r_cylinder); %生成传感器点坐标系

    %%%%%%%%%%%%%%%
    %%% 仿真过程 %%%
    %%%%%%%%%%%%%%%

    %图窗视图全局调整
    % ax = setPlotStyle(scenario);

    %测距对象生成
    chargeStation = robotPlatform("chargeStation"+num2str(idx_config), scenario, InitialBasePosition=[0.4 0 0]);%基底距离将同时影响mesh与实际障碍物。但mesh的高度为中心对称式定义，实际检测物体的高度沿z轴正方向定义。
    chargingStationProfile = struct("Length", 0.176, "Width", 0.149, "Height", 1.57, 'OriginOffset', [0 0 0]);%障碍物实际尺寸，Length为中心对称式，Height为单向式
    chargeStation.updateMesh("Cuboid", Size=[0.176 0.149 1.57*2], Color=[242/255 201/255 187/255]);%mesh尺寸，中心对称式

    jointConfig = q(idx_config,:);%目标关节空间位置
    for i = 1:size_sensor
        tform_sensor = tform_sensor_all{1,i};
        % 超声波传感器模型的建立
        % 基于Automated Driving Toolbox的超声波传感器建立
        ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=tform_sensor(1:3,4)',...%相对于基底的安装位置
            MountingAngles=rad2deg(tform2eul(tform_sensor)),...%相对于基底的安装角度,zyx角，单位为degree
            DetectionRange=[0.009 0.01 0.5], ...%检测范围
            FieldOfView=[25, 25], ...%视场角
            Profiles=chargingStationProfile);%检测目标
        % 转换为Robotics System Toolbox的格式
        ult = robotSensor("UltraSonic"+num2str(i)+"_"+num2str(idx_config), robot, ...
            CustomUltrasonicSensor(ultraSonicSensorModel));%传感器的实际高度。+num2str(i)：防止重名

        setup(scenario);%开始仿真
        move(robot,"joint",jointConfig)%运动
        [~, ~, det, ~] = read(ult);%读取超声波传感器数据

        % plotSimulation(i, scenario, ax, position_helix, spot, tform_sensor_all, tform_sensor, ultraSonicSensorModel, det, robot);

        % advance(scenario);%更新仿真
        % updateSensors(scenario);%更新传感器
        % ultraSonicSensorModel.release();
    end
    % hold off
    restart(scenario);
end