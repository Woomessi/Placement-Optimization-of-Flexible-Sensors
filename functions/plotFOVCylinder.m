function plotFOVCylinder(tform, ultraSonicSensorModel)
%EXAMPLEHELPERPLOTFOVCYLINDER Plot cylinder showing the field of view of
%the sensor
maxDetectionRange = ultraSonicSensorModel.DetectionRange(3);
fov = ultraSonicSensorModel.FieldOfView(1);

[X,Y,Z]=cylinder([0 maxDetectionRange*tan(deg2rad(fov/2))], 50);
h = maxDetectionRange;
Z = Z*h;

M1=makehgtform(yrotate=pi/2);
surf(X,Y,Z, Parent=hgtransform(Matrix=tform*M1), LineStyle='none', FaceAlpha=0.4)
end

