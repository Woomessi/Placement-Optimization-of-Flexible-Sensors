function [spot, tform_sensor_all] = getSpotFrame(size_sensor, position_helix, tform, position, r_cylinder)
% 生成螺旋线均分点（各传感器坐标系原点）
% spot = interparc(size_sensor,position_helix(1,:),position_helix(2,:),position_helix(3,:),'spline');
spot = getUniformPoints(size_sensor, position_helix(1:3,:)', size(position_helix,2)); %自定义函数
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
n = (xk.*xs - x0.*xk - y0.*yk + yk.*ys - z0.*zk + zk.*zs + (r_cylinder.^2.*xk.^2 + r_cylinder.^2.*yk.^2 + r_cylinder.^2.*zk.^2 - x0.^2.*yk.^2 - x0.^2.*zk.^2 + 2.*x0.*xk.*y0.*yk - 2.*x0.*xk.*yk.*ys + 2.*x0.*xk.*z0.*zk - 2.*x0.*xk.*zk.*zs + 2.*x0.*xs.*yk.^2 + 2.*x0.*xs.*zk.^2 - xk.^2.*y0.^2 + 2.*xk.^2.*y0.*ys - xk.^2.*ys.^2 - xk.^2.*z0.^2 + 2.*xk.^2.*z0.*zs - xk.^2.*zs.^2 - 2.*xk.*xs.*y0.*yk + 2.*xk.*xs.*yk.*ys - 2.*xk.*xs.*z0.*zk + 2.*xk.*xs.*zk.*zs - xs.^2.*yk.^2 - xs.^2.*zk.^2 - y0.^2.*zk.^2 + 2.*y0.*yk.*z0.*zk - 2.*y0.*yk.*zk.*zs + 2.*y0.*ys.*zk.^2 - yk.^2.*z0.^2 + 2.*yk.^2.*z0.*zs - yk.^2.*zs.^2 - 2.*yk.*ys.*z0.*zk + 2.*yk.*ys.*zk.*zs - ys.^2.*zk.^2).^(1/2))/(xk.^2 + yk.^2 + zk.^2);
n = real(n);

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
end