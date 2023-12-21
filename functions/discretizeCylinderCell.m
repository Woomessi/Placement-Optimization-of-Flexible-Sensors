function [size_point, point_all] = discretizeCylinderCell(r_cylinder, h_cylinder, size_theta)
interval_h_cylinder = 2*pi*r_cylinder/size_theta; % 圆柱坐标间隔高度
theta0 = 0:(360-0)/(size_theta-1):360; %圆柱坐标角度均分

% 圆柱坐标转换为笛卡尔坐标
x0= r_cylinder*cosd(theta0);
y0= r_cylinder*sind(theta0);
z0 = -h_cylinder:interval_h_cylinder:0;

size_z = size(z0,2); %圆柱坐标高度方向均分数
size_point = size_theta*size_z; %离散点总数

%离散点圆柱坐标保存
point_all = cell(size_point,1); %储存所有离散点圆柱坐标的数组
% point_all = zeros(size_point,2); %储存所有离散点圆柱坐标的数组

k = 1;
for i = 1:size_theta
    for j = 1:size_z
        point_all(k) = {[theta0(i),z0(j)]};
        % point_all(k,:) = [theta0(i),z0(j)];
        k = k+1;
    end
end
end