function plotSimulation(i, scenario, ax, position_helix, spot, tform_sensor_all, tform_sensor, ultraSonicSensorModel, det, robot)
% show3D(scenario,fastUpdate=true,Parent=ax,Collisions="on");
show3D(scenario,Parent=ax);
% drawnow
% 圆柱螺旋线可视化
plot3(position_helix(1,:),position_helix(2,:),position_helix(3,:),'linewidth',1);
% 均分点可视化
scatter3(spot(1,:),spot(2,:),spot(3,:),20,"black","filled")
% 传感器坐标系可视化
quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,1),tform_sensor_all{1,i}(2,1),tform_sensor_all{1,i}(3,1),0.1,"filled",'r')
quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,2),tform_sensor_all{1,i}(2,2),tform_sensor_all{1,i}(3,2),0.1,"filled",'g')
quiver3(tform_sensor_all{1,i}(1,4),tform_sensor_all{1,i}(2,4),tform_sensor_all{1,i}(3,4),tform_sensor_all{1,i}(1,3),tform_sensor_all{1,i}(2,3),tform_sensor_all{1,i}(3,3),0.1,"filled",'b')
% 视场锥可视化
plotFOVCylinder(tform_sensor, ultraSonicSensorModel);

%测距值可视化
if ~isempty(det)
    distance = det{1}.Measurement;%距离值
    % displayText = ['Distance = ',num2str(distance)];
    % t = text(-1, 0, (i-1)*0.1, displayText, "BackgroundColor",'yellow');
    % t(1).Color = 'black';
    % t(1).FontSize = 5;

    % displayText = ['Sensor ',num2str(i),'Distance = ',num2str(distance)];
    displayText = ['Sensor ',num2str(i),': ',num2str(distance),' m'];
    t = text(-1, 0, (i-1)*0.1, displayText, "BackgroundColor",'yellow');
    t(1).Color = 'black';
    t(1).FontSize = 5;

    % config1 = robot.read("joint");
    % Plot a red shpere where the ultrasonic sensor detects an object
    % pose = robot.read();
    % exampleHelperPlotDetectionPoint(scenario, ...
    %     det{1}.ObjectAttributes{1}.PointOnTarget, ...
    %     ult.Name, ...
    %     tform);
    % else
    %     distance = inf;
    %     displayText = 'No object detected!';
end
% t = text(-1, 0, (i-1)*0.1, displayText, "BackgroundColor",'yellow');
% t(1).Color = 'black';
% t(1).FontSize = 5;
% config1 = robot.read("joint");
% quiver3(position(1),position(2),position(3),tform(1,1),tform(2,1),tform(3,1))
% hold off
end