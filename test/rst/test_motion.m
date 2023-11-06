clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))
% addpath(genpath(pwd))

%初始化仿真环境
sampleRate = 10;%与传感器更新频率一致
scenario = robotScenario(UpdateRate=sampleRate, MaxNumFrames=100);%机器人仿真需要较高的帧数

%机器人生成
my_robot = importrobot('model_sensor_compact.urdf');%无mesh，运行更快
% my_robot = importrobot('model_sensor_compact_win.urdf');

robot = robotPlatform("rst", scenario, RigidBodyTree=my_robot);

%RRT路径规划
%机器人关节空间起点与终点定义
initialConfig = homeConfiguration(robot.RigidBodyTree);
pickUpConfig = [0.2371 -0.0200 0.0542 -2.2272 0.0013 ...
    2.2072 -0.9670];
planner = manipulatorRRT(robot.RigidBodyTree,scenario.CollisionMeshes);
planner.IgnoreSelfCollision = true;
rng("default")
path = plan(planner,initialConfig,pickUpConfig);
path = interpolate(planner,path,25);

%仿真
%测距对象生成
chargeStation = robotPlatform("chargeStation", scenario, InitialBasePosition=[0.4 0 0]);%基底距离将同时影响mesh与实际障碍物。但mesh的高度为中心对称式定义，实际检测物体的高度沿z轴正方向定义。
chargingStationProfile = struct("Length", 0.01, "Width", 0.375, "Height", 1.8, 'OriginOffset', [0 0 0]);%障碍物实际尺寸，Length为中心对称式，Height为单向式
chargeStation.updateMesh("Cuboid", Size=[0.01 0.375 3.6], Color=[242/255 201/255 187/255]);%mesh尺寸，中心对称式

config = homeConfiguration(my_robot);%关节空间结构体生成
for idx = 1:size(path,1)%遍历每组关节空间配置
    for idx2 = 1:size(path,2)%遍历每个关节
        config(idx2).JointPosition = path(idx,idx2);
    end
    tform = getTransform(my_robot, config, "Joint5_Link", "base_link");%在当前config下，某Joint_Link坐标系到基底坐标系的齐次变换矩阵
    position = tform(1:3,4)';%Joint_Link坐标系原点在基底坐标系中的坐标
    rpy_zyx = tform2eul(tform);%Joint_Link坐标系姿态相对于基底坐标系的rpy角表示

    %超声波传感器模型的建立
    %基于Automated Driving Toolbox的超声波传感器建立
    ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=position,...%相对于基底的安装位置
        MountingAngles=rad2deg(rpy_zyx),...%相对于基底的安装角度,zyx角，单位为degree
        DetectionRange=[0.009 0.01 0.5], ...%检测范围
        FieldOfView=[25, 25], ...%视场角
        Profiles=chargingStationProfile);%检测目标
    %转换为Robotics System Toolbox的格式
    ult = robotSensor("UltraSonic"+num2str(idx), robot, ...
        CustomUltrasonicSensor(ultraSonicSensorModel));%传感器的实际高度

    setup(scenario);%开始仿真
    [~, ~, det, ~] = read(ult);%读取超声波传感器数据

    figure(1);
    ax = show3D(scenario);
    % view(0,0)%正视
    % view(-90,0)%左视
    % view(90,90)%俯视
    view(-40,15)
    zlim([0 inf])
    light("Position",[-1 -1 0])
    grid on
    hold on

    % 视场锥可视化
    plotFOVCylinder(tform, ultraSonicSensorModel);

    jointConfig = path(idx,:);%目标关节空间位置
    move(robot,"joint",jointConfig)%运动
    % show3D(scenario,fastUpdate=true,Parent=ax,Collisions="on");
    % drawnow

    %测距值可视化
    if ~isempty(det)
        distance = det{1}.Measurement;%距离值
        displayText = ['Distance = ',num2str(distance)];
        % Plot a red shpere where the ultrasonic sensor detects an object
        % pose = robot.read();
        % exampleHelperPlotDetectionPoint(scenario, ...
        %     det{1}.ObjectAttributes{1}.PointOnTarget, ...
        %     ult.Name, ...
        %     tform);
    else
        distance = inf;
        displayText = 'No object detected!';
    end
    t = text(-0.8, 0, 0.4, displayText, "BackgroundColor",'yellow');
    t(1).Color = 'black';
    t(1).FontSize = 10;
    config1 = robot.read("joint");
    quiver3(position(1),position(2),position(3),tform(1,1),tform(2,1),tform(3,1))
    hold off

    advance(scenario);%更新仿真
    updateSensors(scenario);%更新传感器
    ultraSonicSensorModel.release();
end
%Move the joints of the manipulator along the path and visualize the scenario.
% helperRobotMove(path,robot,scenario,ax,ult,ultraSonicSensorModel)