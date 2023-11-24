clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%
%%% 初始化 %%%
%%%%%%%%%%%%%%%
% 连杆三角面片模型输入
% 待修改
TR = stlread('C:\projects\MATLAB\robot_sensor\franka_description\meshes\visual\link6.stl');

position_vertices = TR.Points;% 顶点位置
connectivity_facets = TR.ConnectivityList;% 面片的顶点连接关系

% 待修改
load("all_tform_spot_link6_3.mat");
% all_tform_spot = a;
size_spot = 16; % ToF模块数量

%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%

% for idx_geodesic = 18:18
for idx_geodesic = 18:100:size(all_tform_spot,2)
    tform_spot = all_tform_spot{1,idx_geodesic};

    close all
    hold on
    axis equal
    xlabel("x")
    ylabel("y")
    zlabel("z")
    light("Style","infinite","Position",[0.2 -0.2 0.1])
    view(45,25)
    patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','none','FaceAlpha',.8);
    title("idx geodesic:",num2str(idx_geodesic))
    for i = 1:size_spot
        scatter3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4),18,"magenta",'filled')
        % 绘制拟合路径点坐标系
        quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
            tform_spot{1,i}(1,1),tform_spot{1,i}(2,1),tform_spot{1,i}(3,1),0.05,'Color','r');

        quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
            tform_spot{1,i}(1,2),tform_spot{1,i}(2,2),tform_spot{1,i}(3,2),0.05,'Color','g');

        quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
            tform_spot{1,i}(1,3),tform_spot{1,i}(2,3),tform_spot{1,i}(3,3),0.05,'Color','b');
    end
end