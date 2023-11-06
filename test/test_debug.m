clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%
%%% 初始化 %%%
%%%%%%%%%%%%%%%

% 连杆三角面片模型输入
TR = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link1.stl');
position_vertices = TR.Points;% 顶点位置
connectivity_facets = TR.ConnectivityList;% 面片的顶点连接关系
alpha = 0;% 方向角（弧度制）

% 选择起点
index_facet_current = 6321;% 初始化当前三角面片索引
index_vertices = 8812;% 初始化路径点序号
point_current = position_vertices(index_vertices,:);% 初始化当前路径点
load("dvW_mat.mat");

% 初始化路径点序列
points_seq = point_current;% 路径点序列
indices_facets_seq = index_facet_current;% 三角面片索引序列

% % 建立三角面片局部坐标系
% z_basis = faceNormal(TR,index_facet_current);
% z_basis = z_basis/norm(z_basis);% z基向量
% vertex1 = position_vertices(connectivity_facets(index_facet_current,1),:);% 当前三角面片的第1个顶点
% x_basis = vertex1 - point_current;
% x_basis = x_basis/norm(x_basis);% x基向量
% y_basis = cross(z_basis,x_basis);% y基向量
% T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵
% 
% % 建立方向向量
% direction_vector_F = [cos(alpha),sin(alpha),0,1];% 方向向量（三角面片局部坐标系）。起点为原点时，该变量表示终点
% direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 方向向量（世界坐标系）。[终点 - 起点]方为矢量，故需同时对终点和起点坐标进行齐次变换
% direction_vector_W = direction_vector_W(1:3);


F_color = zeros(size(connectivity_facets,1),1);
F_color(index_facet_current) = 1;
colormap hot
colorbar
axis equal
hold on

% %{
patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceVertexCData',F_color,'FaceColor','flat','EdgeColor','black','FaceAlpha',.8);
scatter3(points_seq(:,1),points_seq(:,2),points_seq(:,3),18,"magenta",'filled')

% quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),0.001,"filled",'r','LineWidth',0.1);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),0.001,"filled",'g','LineWidth',0.1);
% quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),0.001,"filled",'b','LineWidth',0.1);

quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W(1),direction_vector_W(2),direction_vector_W(3),0.001,"filled",'m','LineWidth',0.1);
%}

%%%%%%%%%%%%%%%%%
%%% 更新路径点 %%%
%%%%%%%%%%%%%%%%%

for round = 1

    %%%%%%%%%%%%%%%%
    %%% 计算交点 %%%
    %%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% 判断当前路径点是否为三角面片顶点 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if size(index_vertices,2) == 1
        indices_vertices = connectivity_facets(index_facet_current,:);
        indices_vertices(indices_vertices == index_vertices) = [];
        vertex1 = position_vertices(indices_vertices(1),:);% 当前三角面片的第1个顶点
        vertex2 = position_vertices(indices_vertices(2),:);% 当前三角面片的第2个顶点
        point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex2 - vertex1);
        x_basis = (vertex1 - point_next)/norm(vertex1 - point_next);% 在相交边线上建立新的x轴基向量
        index_vertices = indices_vertices;% 顶点1与顶点2的序号

        % 判断是否到达模型边界
        index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
        index_facets_attached = index_facets_attached{1,1};
        index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
        if size(index_facets_attached,2) == 0
            break
        end

        [direction_vector_W,index_facet_current,point_current,T] = getEdgePathpoint(TR, index_facets_attached, point_next, x_basis, direction_vector_W);
    else 
        %%%%%%%%%%%%
        %%% 否则 %%%
        %%%%%%%%%%%%
        vertex1 = position_vertices(connectivity_facets(index_facet_current,1),:);% 当前三角面片的第1个顶点
        vertex2 = position_vertices(connectivity_facets(index_facet_current,2),:);% 当前三角面片的第2个顶点
        vertex3 = position_vertices(connectivity_facets(index_facet_current,3),:);% 当前三角面片的第3个顶点
        
        % 计算由当前点指向三角面片各顶点的"顶点向量"
        vector_vertex1 = vertex1 - point_current;
        vector_vertex1 = vector_vertex1/norm(vector_vertex1);
        vector_vertex2 = vertex2 - point_current;
        vector_vertex2 = vector_vertex2/norm(vector_vertex2);
        vector_vertex3 = vertex3 - point_current;
        vector_vertex3 = vector_vertex3/norm(vector_vertex3);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 判断方向向量是否与当前三角面片相交于一个顶点 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        if abs(vector_vertex1 - direction_vector_W) < ones(1,3)*0.01% 判断方向向量是否同顶点向量1重合
            point_current = vertex1;% 将顶点1作为新的当前路径点
            % scatter3(point_current(1),point_current(2),point_current(3),18,"magenta",'filled')
            index_vertices = connectivity_facets(index_facet_current,1);% 当前路径点的索引
            [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint(F_color,connectivity_facets,TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
        else
            if abs(vector_vertex2 - direction_vector_W) < ones(1,3)*0.01
                point_current = vertex2;
                index_vertices = connectivity_facets(index_facet_current,2);
                [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint(F_color,connectivity_facets,TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
            else
                if abs(vector_vertex3 - direction_vector_W) < ones(1,3)*0.01
                    point_current = vertex3;
                    index_vertices = connectivity_facets(index_facet_current,3);
                    [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint(F_color,connectivity_facets,TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                else
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %%% 否则，方向向量与当前三角面片边线中间的点相交 %%%
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    % 计算各顶点向量的夹角
                    angle_12 = getAngle(vector_vertex1,vector_vertex2);
                    angle_13 = getAngle(vector_vertex1,vector_vertex3);
                    angle_23 = getAngle(vector_vertex2,vector_vertex3);

                    % 计算方向向量与各顶点向量的夹角
                    angle_direction_vector_1 = getAngle(vector_vertex1,direction_vector_W);
                    angle_direction_vector_2 = getAngle(vector_vertex2,direction_vector_W);
                    angle_direction_vector_3 = getAngle(vector_vertex3,direction_vector_W);

                    % 如果方向向量与顶点向量1与顶点向量2的夹角之和等于顶点向量1与顶点向量2的夹角
                    if abs(angle_12 - (angle_direction_vector_1 + angle_direction_vector_2)) <= 0.01 && abs(angle_12 - pi) >= 0.01
                        % 说明方向向量相交于顶点1与顶点2之间的边线，将交点作为新的路径点
                        point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex2 - vertex1);
                        x_basis = (vertex1 - point_next)/norm(vertex1 - point_next);% 在相交边线上建立新的x轴基向量
                        index_vertices = [connectivity_facets(index_facet_current,1),connectivity_facets(index_facet_current,2)];% 顶点1与顶点2的序号

                        % 判断是否到达模型边界
                        index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                        index_facets_attached = index_facets_attached{1,1};
                        index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                        if size(index_facets_attached,2) == 0
                            break
                        end

                        [direction_vector_W,index_facet_current,point_current,T] = getEdgePathpoint(TR, index_facets_attached, point_next, x_basis, direction_vector_W);
                    else
                        if abs(angle_13 - (angle_direction_vector_1 + angle_direction_vector_3)) <= 0.01 && abs(angle_13 - pi) >= 0.01
                            point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex3 - vertex1);
                            x_basis = (vertex3 - point_next)/norm(vertex3 - point_next);
                            index_vertices = [connectivity_facets(index_facet_current,1),connectivity_facets(index_facet_current,3)];

                            % 判断是否到达模型边界
                            index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                            index_facets_attached = index_facets_attached{1,1};
                            index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                            if size(index_facets_attached,2) == 0
                                break
                            end

                            [direction_vector_W,index_facet_current,point_current,T] = getEdgePathpoint(TR, index_facets_attached, point_next, x_basis, direction_vector_W);
                        else
                            point_next = getIntersection(point_current,direction_vector_W,vertex2,vertex3 - vertex2);
                            x_basis = (vertex2 - point_next)/norm(vertex2 - point_next);
                            index_vertices = [connectivity_facets(index_facet_current,2),connectivity_facets(index_facet_current,3)];

                            % 判断是否到达模型边界
                            index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                            index_facets_attached = index_facets_attached{1,1};
                            index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                            if size(index_facets_attached,2) == 0
                                break
                            end

                            [direction_vector_W,index_facet_current,point_current,T] = getEdgePathpoint(TR, index_facets_attached, point_next, x_basis, direction_vector_W);
                        end
                    end
                end
            end
        end
    end

    % 更新路径信息
    % round = round + 1;
    points_seq(round,:) = point_current;
    indices_facets_seq(round) = index_facet_current;
    F_color(index_facet_current) = 1;
    
    % %{
    patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceVertexCData',F_color,'FaceColor','flat','EdgeColor','black','FaceAlpha',.8);
    scatter3(points_seq(:,1),points_seq(:,2),points_seq(:,3),18,"magenta",'filled')

    quiver3(T(1,4),T(2,4),T(3,4),T(1,1),T(2,1),T(3,1),0.001,"filled",'r','LineWidth',0.1);
    quiver3(T(1,4),T(2,4),T(3,4),T(1,2),T(2,2),T(3,2),0.001,"filled",'g','LineWidth',0.1);
    quiver3(T(1,4),T(2,4),T(3,4),T(1,3),T(2,3),T(3,3),0.001,"filled",'b','LineWidth',0.1);

    quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W(1),direction_vector_W(2),direction_vector_W(3),0.001,"filled",'m','LineWidth',0.1);
    %}
end

patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceVertexCData',F_color,'FaceColor','flat','EdgeColor','black','FaceAlpha',.8);
scatter3(points_seq(:,1),points_seq(:,2),points_seq(:,3),18,"magenta",'filled')