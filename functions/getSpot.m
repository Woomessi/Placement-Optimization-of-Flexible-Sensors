function [position_helix, spot, tform_sensor_all] = getSpot(edge_candidate, idx_geodesic, point_all, edge, r_cylinder, tform, size_sensor, position)
if edge_candidate(idx_geodesic,1)==0 %共轭测地线
    idx = edge_candidate(idx_geodesic,2);
    
    theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
    z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
    theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
    z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标
    %生成圆柱螺旋线笛卡尔坐标
    xi1 = theta1+2*pi;  xi2 = theta2;
    u = linspace(xi1,xi2,500);
    x = r_cylinder*cos(u);
    y = r_cylinder*sin(u);
    z = (z2-z1)/(xi2-xi1)*u + (z1*xi2-z2*xi1)/(xi2-xi1);
else
    idx = edge_candidate(idx_geodesic,1); %测地线

    theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
    z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
    theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
    z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标

    % 生成圆柱螺旋线笛卡尔坐标
    u = linspace(theta1,theta2,500); %均分角度
    x = r_cylinder*cos(u);
    y = r_cylinder*sin(u);
    z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
end

position_helix_local = [x;y;z;ones(1,500)]; %当地坐标系下的螺旋线笛卡尔坐标

position_helix = tform*position_helix_local; %世界坐标系下的螺旋线笛卡尔坐标

% 生成螺旋线均分点（各传感器坐标系原点）
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