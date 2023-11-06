clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

% 连杆三角面片模型输入
TR = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link1.stl');
position_vertices = TR.Points;% 顶点位置
facets = TR.ConnectivityList;% 面片的顶点连接关系
alpha = 0;% 方向角

% 选择起点
index_facet_current = 4631;% 当前三角面片索引
point_current = incenter(TR,index_facet_current);% 当前路径点

% 初始化路径点序列
points_seq = point_current;% 路径点序列
indices_facets_seq = index_facet_current;% 三角面片索引序列
round = 1; % 路径点编号
vertex1 = position_vertices(facets(index_facet_current,1),:);% 当前三角面片的第1个顶点
vertex2 = position_vertices(facets(index_facet_current,2),:);% 当前三角面片的第2个顶点
vertex3 = position_vertices(facets(index_facet_current,3),:);% 当前三角面片的第3个顶点

% 建立三角面片局部坐标系
z_basis = faceNormal(TR,index_facet_current);
z_basis = z_basis/norm(z_basis);% z基向量
x_basis = vertex1 - point_current;
x_basis = x_basis/norm(x_basis);% x基向量
y_basis = cross(z_basis,x_basis);% y基向量
T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

% 建立方向向量
direction_vector_F = [cos(alpha),sin(alpha),0,1];% 方向向量（三角面片局部坐标系）。起点为原点时，该变量表示终点
direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 方向向量（世界坐标系）。[终点 - 起点]方为矢量，故需同时对终点和起点坐标进行齐次变换
direction_vector_W = direction_vector_W(1:3);

% %{
% 作图
% 经过的三角面片
F_color = zeros(size(facets,1),1);
F_color(index_facet_current) = 1;
patch('Faces',facets,'Vertices',position_vertices,'FaceVertexCData',F_color,'FaceColor','flat');
colormap hot
colorbar
axis equal
hold on
% 当前点
scatter3(point_current(1),point_current(2),point_current(3),18,"magenta",'filled')
% 三角面片局部坐标系
quiver3(T(1,4),T(2,4),T(3,4),x_basis(1),x_basis(2),x_basis(3),0.001,"filled",'r','LineWidth',0.1);
quiver3(T(1,4),T(2,4),T(3,4),y_basis(1),y_basis(2),y_basis(3),0.001,"filled",'g','LineWidth',0.1);
quiver3(T(1,4),T(2,4),T(3,4),z_basis(1),z_basis(2),z_basis(3),0.001,"filled",'b','LineWidth',0.1);
% 方向向量
quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W(1),direction_vector_W(2),direction_vector_W(3),0.001,"filled",'m','LineWidth',0.1);
%}

%%%%%%%%%%%%%%%%
%%% 计算交点 %%%
%%%%%%%%%%%%%%%%

% 计算由当前点指向三角面片各顶点的"顶点向量"
vector_vertex1 = vertex1 - point_current;
vector_vertex1 = vector_vertex1/norm(vector_vertex1);
vector_vertex2 = vertex2 - point_current;
vector_vertex2 = vector_vertex2/norm(vector_vertex2);
vector_vertex3 = vertex3 - point_current;
vector_vertex3 = vector_vertex3/norm(vector_vertex3);

% 更新当前路径点
if abs(vector_vertex1 - direction_vector_W) < ones(1,3)*0.01% 判断方向向量是否同顶点向量重合？
    point_current = vertex1;% 将顶点1作为新的当前路径点
    index_point_current = facets(index_facet_current,1);% 当前路径点的索引
else
    if abs(vector_vertex2 - direction_vector_W) < ones(1,3)*0.01
        point_current = vertex2;
        index_point_current = facets(index_facet_current,2);
    else
        if abs(vector_vertex3 - direction_vector_W) < ones(1,3)*0.01
            point_current = vertex3;
            index_point_current = facets(index_facet_current,3);
        else
        end
    end
end

% 计算均分角
facets_attached = vertexAttachments(TR,index_point_current);% 下一个路径点所在的所有三角面片
facets_attached = facets_attached{1,1};
vectors_edge_point_next = zeros(3,3);
angle_sum = 0;% 初始化总顶点角
for i = 1:size(facets_attached,2)% 遍历下一个路径点所在的所有三角面片
    % 当前三角面片中由其它顶点指向下一个路径点的边向量
    vectors_edge_point_next(1,:) = position_vertices(facets(facets_attached(i),1),:) - point_current;
    vectors_edge_point_next(2,:) = position_vertices(facets(facets_attached(i),2),:) - point_current;
    vectors_edge_point_next(3,:) = position_vertices(facets(facets_attached(i),3),:) - point_current;
    % 去除与下一个路径点重合的顶点产生的边向量
    vectors_edge_point_next(all(vectors_edge_point_next == 0,2),:) = [];
    % 将当前面片的顶点角加入总顶点角中
    angle_sum = angle_sum + getAngle(vectors_edge_point_next(1,:),vectors_edge_point_next(2,:));
end
phi = angle_sum/2;% 均分角

%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 寻找下一个方向向量 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

% 任选当前三角面片的一个其余顶点，与当前路径点组成相邻边
index_vertice_neighbor = facets(index_facet_current,:);% 当前三角面片的所有顶点序号
index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];% 删除下一个路径点的序号
index_vertice_neighbor = index_vertice_neighbor(1);% 任选当前三角面片的一个其余顶点

vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);% 相邻边线的向量
phi_sum = getAngle(direction_vector_W,vector_edge_neighbor);% 计算当前相邻边与方向向量的夹角

index_last_facet_neighbor = index_facet_current;% 保存当前三角面片的序号

%%%%%%%%%%%%%
%%% 循环1 %%%
%%%%%%%%%%%%%
while phi_sum < phi
% 下一个相邻三角面片
index_facet_neighbor = edgeAttachments(TR,index_point_current,index_vertice_neighbor);% 相邻边所在的两个三角面片的序号
index_facet_neighbor = index_facet_neighbor{1,1};
index_facet_neighbor(index_facet_neighbor == index_last_facet_neighbor) = [];% 删除上一个三角面片的序号

% 下一个相邻顶点
index_last_vertice_neighbor = index_vertice_neighbor;% 保存上一个相邻顶点
index_vertice_neighbor = facets(index_facet_neighbor,:);% 当前三角面片的所有顶点序号
index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];% 删除下一个路径点的序号
index_vertice_neighbor(index_vertice_neighbor == index_last_vertice_neighbor) = [];% 删除上一个相邻顶点的序号

% 下一个相邻边
vector_last_edge_neighbor = vector_edge_neighbor;% 保存上一个相邻边向量
vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);% 计算当前相邻边
phi_sum = phi_sum + getAngle(vector_last_edge_neighbor,vector_edge_neighbor);% 计算当前相邻边与上一个相邻边的夹角

index_last_facet_neighbor = index_facet_neighbor;% 保存当前三角面片的序号
end
%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 确定下一个方向向量 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%

phi_sum = phi_sum - getAngle(vector_last_edge_neighbor,vector_edge_neighbor);% 退回到上一轮所计算的累加夹角
alpha = phi - phi_sum;% 将均分角与累加夹角之差作为新的方向角
index_facet_current = index_facet_neighbor;% 将本轮所搜寻的相邻面片序号作为当前三角面片的序号

% 更新路径信息
round = round + 1;
points_seq(round,:) = point_current;
indices_facets_seq(round) = index_facet_current;

% 建立新的三角面片局部坐标系
z_basis = faceNormal(TR,index_facet_current);
z_basis = z_basis/norm(z_basis);
x_basis = -vector_last_edge_neighbor;% 将上一个相邻边向量的反方向作为x基向量的方向
x_basis = x_basis/norm(x_basis);
y_basis = cross(z_basis,x_basis);
T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

% 保证方向向量向前
if dot(direction_vector_W,y_basis) < 0
    alpha = -alpha;
end

% 建立方向向量
direction_vector_F = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
direction_vector_W = direction_vector_W(1:3);


% %{
% 作图
F_color(index_facet_current) = 1;
patch('Faces',facets,'Vertices',position_vertices,'FaceVertexCData',F_color,'FaceColor','flat');

scatter3(point_current(1),point_current(2),point_current(3),18,"magenta",'filled')

quiver3(T(1,4),T(2,4),T(3,4),x_basis(1),x_basis(2),x_basis(3),0.001,"filled",'r','LineWidth',0.1);
quiver3(T(1,4),T(2,4),T(3,4),y_basis(1),y_basis(2),y_basis(3),0.001,"filled",'g','LineWidth',0.1);
quiver3(T(1,4),T(2,4),T(3,4),z_basis(1),z_basis(2),z_basis(3),0.001,"filled",'b','LineWidth',0.1);

quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W(1),direction_vector_W(2),direction_vector_W(3),0.001,"filled",'m','LineWidth',0.1);
%}