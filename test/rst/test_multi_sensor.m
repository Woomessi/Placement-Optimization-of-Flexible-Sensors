clear
clc
close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%初始化仿真环境
sampleRate = 10;%与传感器更新频率一致
scenario = robotScenario(UpdateRate=sampleRate, MaxNumFrames=100);%机器人仿真需要较高的帧数
size_sensor = 16;

%机器人生成
% my_robot = importrobot('model_sensor_compact.urdf');%无mesh，运行更快
my_robot = importrobot('model_sensor_compact_win.urdf');

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

%测距对象生成
chargeStation = robotPlatform("chargeStation", scenario, InitialBasePosition=[0.4 0 0]);%基底距离将同时影响mesh与实际障碍物。但mesh的高度为中心对称式定义，实际检测物体的高度沿z轴正方向定义。
chargingStationProfile = struct("Length", 0.176, "Width", 0.149, "Height", 1.57, 'OriginOffset', [0 0 0]);%障碍物实际尺寸，Length为中心对称式，Height为单向式
chargeStation.updateMesh("Cuboid", Size=[0.176 0.149 1.57*2], Color=[242/255 201/255 187/255]);%mesh尺寸，中心对称式

config = homeConfiguration(my_robot);%关节空间结构体生成
% for idx = 1:size(path,1)%遍历每组关节空间配置
idx = 32;%调整当前关节角配置
for idx2 = 1:size(path,2)%遍历每个关节
    config(idx2).JointPosition = path(idx,idx2);
end

%坐标变换
tform = getTransform(my_robot, config, "Joint5_Link", "base_link");%在当前config下，某Joint_Link坐标系到基底坐标系的齐次变换矩阵
position = tform(1:3,4)';%Joint_Link坐标系原点在基底坐标系中的坐标
rpy_zyx = tform2eul(tform);%Joint_Link坐标系姿态相对于基底坐标系的rpy角表示

% 圆柱螺旋线设置
r = 0.043;%圆柱半径
% 生成起点和终点坐标
theta1= 0*pi/180;     z1= -0.02;
theta2= 225*pi/180;    z2= -0.06;
% x1= r*cos(theta1);  y1= r*sin(theta1);
% x2= r*cos(theta2);  y2= r*sin(theta2);
% 生成圆柱螺旋线坐标
u = linspace(theta1,theta2,500);%均分角度
x = r*cos(u); y = r*sin(u);
z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
position_helix_local = [x;y;z;ones(1,500)];%当地坐标系下的坐标
position_helix = tform*position_helix_local;%世界坐标系下的坐标
% 生成均分点
spot = interparc(size_sensor,position_helix(1,:),position_helix(2,:),position_helix(3,:),'spline');
spot = spot';
xs = spot(1,:);
ys = spot(2,:);
zs = spot(3,:);
% Z轴方向向量
z_basis = tform(1:3,3);
xk = z_basis(1);
yk = z_basis(2);
zk = z_basis(3);
% 局部坐标系原点
x0 = position(1);
y0 = position(2);
z0 = position(3);
% 各均分点对应圆心
n = (xk.*xs - x0.*xk - y0.*yk + yk.*ys - z0.*zk + zk.*zs + (r.^2.*xk.^2 + r.^2.*yk.^2 + r.^2.*zk.^2 - x0.^2.*yk.^2 - x0.^2.*zk.^2 + 2.*x0.*xk.*y0.*yk - 2.*x0.*xk.*yk.*ys + 2.*x0.*xk.*z0.*zk - 2.*x0.*xk.*zk.*zs + 2.*x0.*xs.*yk.^2 + 2.*x0.*xs.*zk.^2 - xk.^2.*y0.^2 + 2.*xk.^2.*y0.*ys - xk.^2.*ys.^2 - xk.^2.*z0.^2 + 2.*xk.^2.*z0.*zs - xk.^2.*zs.^2 - 2.*xk.*xs.*y0.*yk + 2.*xk.*xs.*yk.*ys - 2.*xk.*xs.*z0.*zk + 2.*xk.*xs.*zk.*zs - xs.^2.*yk.^2 - xs.^2.*zk.^2 - y0.^2.*zk.^2 + 2.*y0.*yk.*z0.*zk - 2.*y0.*yk.*zk.*zs + 2.*y0.*ys.*zk.^2 - yk.^2.*z0.^2 + 2.*yk.^2.*z0.*zs - yk.^2.*zs.^2 - 2.*yk.*ys.*z0.*zk + 2.*yk.*ys.*zk.*zs - ys.^2.*zk.^2).^(1/2))/(xk.^2 + yk.^2 + zk.^2); 
n = real(n);
% test =[x0;y0;z0];
% test2 = spot - n.*z_basis;
cs = spot - n.*z_basis - [x0;y0;z0];
% 传感器坐标系X轴方向向量
x_basis = zeros(3,size_sensor);
for i = 1:size_sensor
    x_basis(:,i) = cs(:,i)/norm(cs(:,i));
end
% 传感器坐标系Y轴方向向量
y_basis = zeros(3,size_sensor);
for i = 1:size_sensor
    y_basis(:,i) = cross(z_basis, x_basis(:,i));
end
% 各传感器坐标系齐次变换矩阵
tform_sensor_all = cell(1,size_sensor);
for i = 1:size_sensor
    tform_sensor_all(1,i) = {[x_basis(:,i), y_basis(:,i) ,z_basis, spot(:,i); 0 0 0 1]};
end

%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% 仿真 %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%
%图窗视图全局调整
figure(1);
ax = show3D(scenario);
view(0,0)%正视
% view(-90,0)%左视
% view(90,90)%俯视

% view(-40,15)
zlim([0 inf])

% ax.View = [-37.5,30];
% ax.CameraPositionMode = "manual";
% ax.CameraPosition = [-6.833607033563605,-8.948799195144693,7.388762690675049];
% ax.CameraTargetMode = "manual";
% ax.CameraTarget = [0.176872113602082,0.187435105215806,0.740013558596934];
% ax.CameraViewAngleMode = "manual";
% ax.CameraViewAngle = 1.347371145846553;
% ax.XLim = [-1.933024686348753,2.631392491565355];
% ax.YLim = [-1.414744027057242,2.185255972942756];
% ax.ZLim = [-1.246737247414615,2.353262752585382];

light("Position",[-1 -1 0])
grid on
hold on

for idx3 = 1:size_sensor
tform_sensor = tform_sensor_all{1,idx3};
% 超声波传感器模型的建立
% 基于Automated Driving Toolbox的超声波传感器建立
ultraSonicSensorModel = ultrasonicDetectionGenerator(MountingLocation=tform_sensor(1:3,4)',...%相对于基底的安装位置
    MountingAngles=rad2deg(tform2eul(tform_sensor)),...%相对于基底的安装角度,zyx角，单位为degree
    DetectionRange=[0.009 0.01 0.5], ...%检测范围
    FieldOfView=[25, 25], ...%视场角
    Profiles=chargingStationProfile);%检测目标
% 转换为Robotics System Toolbox的格式
ult = robotSensor("UltraSonic"+num2str(idx3), robot, ...
    CustomUltrasonicSensor(ultraSonicSensorModel));%传感器的实际高度

setup(scenario);%开始仿真
[~, ~, det, ~] = read(ult);%读取超声波传感器数据

% 起点可视化
scatter3(tform_sensor(1,4),tform_sensor(2,4),tform_sensor(3,4),100,"filled","MarkerFaceColor","#00FFFF")
% Z轴可视化
% quiver3(tform_sensor(1,4),tform_sensor(2,4),tform_sensor(3,4),tform_sensor(1,3),tform_sensor(2,3),tform_sensor(3,3),1)
% 圆柱螺旋线可视化
plot3(position_helix(1,:),position_helix(2,:),position_helix(3,:),'linewidth',1);
% 均分点可视化
scatter3(spot(1,:),spot(2,:),spot(3,:),9,"magenta","filled")
% 传感器坐标系可视化
for i = 1:size_sensor
    quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,1),tform_sensor_all{1,i}(2,1),tform_sensor_all{1,i}(3,1),0.1,"filled",'r')
    quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,2),tform_sensor_all{1,i}(2,2),tform_sensor_all{1,i}(3,2),0.1,"filled",'g')
    quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,3),tform_sensor_all{1,i}(2,3),tform_sensor_all{1,i}(3,3),0.1,"filled",'b')
end
% 视场锥可视化
plotFOVCylinder(tform_sensor, ultraSonicSensorModel);

jointConfig = path(idx,:);%目标关节空间位置
move(robot,"joint",jointConfig)%运动
% show3D(scenario,fastUpdate=true,Parent=ax,Collisions="on");
show3D(scenario,Parent=ax);
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
t = text(-1, 0, (idx3-1)*0.1, displayText, "BackgroundColor",'yellow');
t(1).Color = 'black';
t(1).FontSize = 5;
config1 = robot.read("joint");
% quiver3(position(1),position(2),position(3),tform(1,1),tform(2,1),tform(3,1))
% hold off

advance(scenario);%更新仿真
updateSensors(scenario);%更新传感器
ultraSonicSensorModel.release();
end