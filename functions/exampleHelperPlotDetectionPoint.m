function exampleHelperPlotDetectionPoint(scenario, pointOnTarget, sensorName, tform)
%EXAMPLEHELPERPLOTDETECTIONPOINT Plot detection point as red sphere

detectionPtWRTSensor = pointOnTarget; %wrt sensor
sensorENUTform = scenario.TransformTree.getTransform("ENU", sensorName);
R = tform(1:3,1:3); %rotm of the platform
detectionPtWRTPlatform = R*detectionPtWRTSensor; %wrt platform
detectionPtWRTENU = sensorENUTform(1:3,4) + detectionPtWRTPlatform; %wrt world frame

[X,Y,Z] = sphere;
r = 0.01;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

% surf(X2+detectionPtWRTENU(1), ...
%     Y2+detectionPtWRTENU(2), ...
%     Z2+pose(3)+.25, FaceColor='r', EdgeColor='r');

surf(X2+detectionPtWRTENU(1), ...
    Y2+detectionPtWRTENU(2), ...
    Z2+detectionPtWRTENU(3), FaceColor='r', EdgeColor='r');
end
 
