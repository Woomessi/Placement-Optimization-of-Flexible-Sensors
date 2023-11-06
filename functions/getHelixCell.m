function [edge, size_edge, l_edge] = getHelixCell(size_point, point_all, r_cylinder)
%基于图的测地线长度计算
% tic
% adjacent_matrix = ones(size_point) - eye(size_point); %邻接矩阵
adjacent_matrix = ones(size_point); %允许闭环的邻接矩阵
graph_point = graph(adjacent_matrix); %离散点连接关系图
edge = graph_point.Edges.EndNodes; %离散点对所有组合（不重复）
size_edge = size(graph_point.Edges,1); %离散点对总数

l_edge = zeros(size_edge,2); %储存两离散点间测地线长度的数组
for i = 1:size_edge
    theta1 = point_all{edge(i,1)}(1); %起点角度圆柱坐标
    z1 = point_all{edge(i,1)}(2); %起点高度圆柱坐标
    theta2 = point_all{edge(i,2)}(1); %终点角度圆柱坐标
    z2 = point_all{edge(i,2)}(2); %终点高度圆柱坐标

    % theta1 = point_all(edge(i,1),1); %起点角度圆柱坐标
    % z1 = point_all(edge(i,1),2); %起点高度圆柱坐标
    % theta2 = point_all(edge(i,2),1); %终点角度圆柱坐标
    % z2 = point_all(edge(i,2),2); %终点高度圆柱坐标

    l_edge(i,1) = sqrt(r_cylinder^2*(theta2-theta1)^2+(z2-z1)^2); %测地线长度计算
    
%共轭螺旋线
    xi1 = theta1+2*pi;  xi2 = theta2;
    l_edge(i,2) = sqrt(r_cylinder^2*(xi2-xi1)^2+(z2-z1)^2);
end
% toc

%传统长度计算方法
% length_all = zeros(size_point);
% tic
% for i = 1:size_point
%     for j = 1:size_point
%         theta1 = point_all{i,1}(1);
%         z1 = point_all{i,1}(2);
%         theta2 = point_all{j,1}(1);
%         z2 = point_all{j,1}(2);
%         length_all(i,j) = sqrt(r^2*(theta2-theta1)^2+(z2-z1)^2); %螺旋线长度
%     end
% end
% toc
end