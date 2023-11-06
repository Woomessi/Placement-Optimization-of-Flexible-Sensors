function helperRobotMove(path,robot,scenario,ax,ult,ultraSonicSensorModel)
for idx = 1:size(path,1)
    jointConfig = path(idx,:);
    move(robot,"joint",jointConfig)
    show3D(scenario,fastUpdate=true,Parent=ax,Collisions="on");
    drawnow
    pose = robot.read();
    rotAngle = quat2eul(pose(10:13));
    hold on

    [~, ~, det, ~] = read(ult);
    if ~isempty(det)
        % Distance to object
        distance = det{1}.Measurement;
        displayText = ['Distance = ',num2str(distance)];
    else
        distance = inf;
        displayText = 'No object detected!';
    end

    % Plot a cone to represent the field of view and range of the ultrasonic sensor
    plotFOVCylinder(pose, ultraSonicSensorModel.DetectionRange(3), ultraSonicSensorModel.FieldOfView(1));
    hold off

    % Display the distance to the charging station detected by the ultrasonic sensor
    t = text(0, 0, displayText, "BackgroundColor",'yellow');
    t(1).Color = 'black';
    t(1).FontSize = 10;

    advance(scenario);
    updateSensors(scenario);
end
end