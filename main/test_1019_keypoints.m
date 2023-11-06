clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%
%%% 初始化 %%%
%%%%%%%%%%%%%%%

% 连杆三角面片模型输入
TR = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link1.stl');
position_vertices = TR.Points;% 顶点位置
connectivity_facets = TR.ConnectivityList;% 面片的顶点连接关系

% 连杆mesh模型几何信息提取
N_f = faceNormal(TR); % 计算三角面法向量
N_v = vertexNormal(TR); % 计算顶点法向量
C = incenter(TR); % 计算三角面形心
X_refined = [position_vertices;C]; % 向顶点坐标矩阵中加入三角面形心坐标
N_refined = [N_v;N_f]; % 向顶点法向量矩阵中加入三角面法向量，作为三角面片形心法向量

load("all_points_seq_link1.mat")
load("index_geodesic.mat")

size_sensor = 16;
length_desired = 0.27;
interval_segment = length_desired/(size_sensor-1);

size_all_points_seq = index_geodesic-1;
all_points_seq = all_points_seq(1,1:size_all_points_seq);
all_tform_spot = cell(1,size_all_points_seq);

all_spot = cell(1,size_all_points_seq);
all_first_spot = zeros(3,size_all_points_seq);
all_middle_spot = zeros(3,size_all_points_seq);
all_last_spot = zeros(3,size_all_points_seq);


for idx_geodesic = 1:size_all_points_seq
    %%%%%%%%%%%%%%%%%
    %%% 均分点提取 %%%
    %%%%%%%%%%%%%%%%%
    points_seq_current = all_points_seq{1,idx_geodesic};
    size_points_geodesic = size(points_seq_current,2);

    % length = 0;
    % for i = 1:size_points_geodesic-1
    %     vector = points_seq_current(:,i+1) - points_seq_current(:,i);
    %     length = length + norm(vector);
    % end

    % 初始化
    length_segment = 0;
    length_segment_sum = 0;
    flag = 0;
    spot = zeros(3,size_sensor);% 初始化均分点
    y_point_path = zeros(3,size_sensor);

    % 生成第一个均分点
    spot(:,1) = points_seq_current(:,1);
    % 初步选定相邻路径点之间的连线为坐标系的y轴基向量
    y_point_path(:,1) = points_seq_current(:,2) - points_seq_current(:,1);
    y_point_path(:,1) = y_point_path(:,1)./norm(y_point_path(:,1));

    % 生成最后一个均分点
    spot(:,size_sensor) = points_seq_current(:,size_points_geodesic);
    y_point_path(:,size_sensor) = points_seq_current(:,size_points_geodesic) - points_seq_current(:,size_points_geodesic-1);
    y_point_path(:,size_sensor) = y_point_path(:,size_sensor)./norm(y_point_path(:,size_sensor));

    % 生成其余的均分点
    i = 1;
    j = 2;
    while spot(:,size_sensor-1) == zeros(3,1) % 当均分点尚未补充完整时
        if flag == 0 % 尚未达到分段长度标准
            p2 = points_seq_current(:,i+1);
            p1 = points_seq_current(:,i);
        else
            p2 = points_seq_current(:,i+1);
            p1 = spot(:,j-1);
            flag = 0;
        end
        vector1 = p2 - p1;
        length_segment = length_segment + norm(vector1);

        if length_segment > interval_segment % 达到分段长度标准时
            % 调整分段点，使长度刚好满足要求
            t = 1-sqrt((norm(vector1)-length_segment+interval_segment).^2*((p1(1)-p2(1)).^2+(p1(2)-p2(2)).^2+(p1(3)-p2(3)).^2))./((p1(1)-p2(1)).^2+(p1(2)-p2(2)).^2+(p1(3)-p2(3)).^2);
            spot(:,j) = t*p1+(1-t)*p2;
            y_point_path(:,j) = vector1./norm(vector1);

            length_segment = 0;
            j = j+1;
            flag = 1;
            length_segment_sum = length_segment_sum + interval_segment;
        else
            i = i+1;
        end
    end
    all_spot{1,idx_geodesic} = spot;
    all_first_spot(:,idx_geodesic) = spot(:,1);
    all_middle_spot(:,idx_geodesic) = spot(:,8);
    all_last_spot(:,idx_geodesic) = spot(:,size_sensor);

    %{
% 拟合路径点坐标系建立
% 在三角面片顶点与形心点中提取距各拟合路径点最近的点
ID_all = dsearchn(X_refined,spot');
% ID_all = dsearchn(C,spot');
% 将各最近点的法向量作为相应路径点的法向量 (x轴基向量)
x_point_path = N_refined(ID_all,:)';
% x_point_path = N_f(ID_all,:)';

% 通过叉乘获得z轴基向量
z_point_path = cross(x_point_path,y_point_path);
z_point_path = z_point_path./vecnorm(z_point_path);

% 通过叉乘修正y轴基向量
y_point_path = cross(z_point_path,x_point_path);

tform_spot_current = cell(1,size_sensor);
for i = 1:size_sensor
    tform_spot_current{1,i} = [x_point_path(:,i),y_point_path(:,i),z_point_path(:,i),spot(:,i);0,0,0,1];
end
all_tform_spot{1,idx_geodesic} = tform_spot_current;
    %}
    %%%%%%%%%%%%
    %%% 作图 %%%
    %%%%%%%%%%%%
    % axis equal
    % xlabel("x")
    % ylabel("y")
    % zlabel("z")
    % light("Style","infinite","Position",[0.2 -0.2 0.1])
    % view(45,25)
    % hold on
    %
    % patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','none','FaceAlpha',.8);
    % scatter3(spot(1,:),spot(2,:),spot(3,:),18,"magenta",'filled')
    % % scatter3(points_seq_current(1,:),points_seq_current(2,:),points_seq_current(3,:),18,"cyan",'filled')
    %
    % % 绘制拟合路径点坐标系
    % quiver3(spot(1,:),spot(2,:),spot(3,:), ...
    %     x_point_path(1,:),x_point_path(2,:),x_point_path(3,:),0.5,'Color','r');
    %
    % quiver3(spot(1,:),spot(2,:),spot(3,:), ...
    %     y_point_path(1,:),y_point_path(2,:),y_point_path(3,:),0.5,'Color','g');
    %
    % quiver3(spot(1,:),spot(2,:),spot(3,:), ...
    %     z_point_path(1,:),z_point_path(2,:),z_point_path(3,:),0.5,'Color','b');
    %
    % hold off
end