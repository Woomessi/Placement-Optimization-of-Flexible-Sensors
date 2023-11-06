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

load("all_points_seq_link1.mat")
load("index_geodesic.mat")

size_all_points_seq = index_geodesic-1;
all_points_seq = all_points_seq(1,1:size_all_points_seq);
load("idx_geodesic.mat")

%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%

axis equal
xlabel("x")
ylabel("y")
zlabel("z")
light("Style","infinite","Position",[0.2 -0.2 0.1])
view(45,25)
hold on
for i = 1:size(idx_geodesic,2)
    points_seq_current = all_points_seq{1,idx_geodesic(i)};
    % patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','white','FaceAlpha',.8);
    patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','none','FaceAlpha',.8);
    scatter3(points_seq_current(1,:),points_seq_current(2,:),points_seq_current(3,:),18,"magenta",'filled')
end