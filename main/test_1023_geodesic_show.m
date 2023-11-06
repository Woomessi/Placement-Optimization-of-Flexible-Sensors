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
load("all_tform_spot_link1.mat");
size_spot = 16; % ToF模块数量

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
% for idx_geodesic = 2911:2911
for idx_geodesic = 16966:16966 % link1 GA 
% for idx_geodesic = 11529:11529 % link1 Bayes
% for idx_geodesic = 10303:10303 % link2 GA

    tform_spot = all_tform_spot{1,idx_geodesic};
    % patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','white','FaceAlpha',.8);
    patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','none','FaceAlpha',.8);
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