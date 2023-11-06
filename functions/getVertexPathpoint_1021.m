function [direction_vector_W,index_facet_current,point_current,T,flag_empty] = getVertexPathpoint_1021(TR, index_point_current, position_vertices, facets, point_current, index_facet_current, direction_vector_W)
flag_empty = 0;
%%%%%%%%%%%%%%%%%
%%% 计算均分角 %%%
%%%%%%%%%%%%%%%%%
facets_attached = vertexAttachments(TR,index_point_current);% 新路径点所在的所有三角面片
facets_attached = facets_attached{1,1};
vectors_edge_point_next = zeros(3,3);
angle_sum = 0;% 初始化总顶点角
for i = 1:size(facets_attached,2)% 遍历新路径点所在的所有三角面片
    % 当前三角面片中由其它顶点指向新路径点的边向量
    vectors_edge_point_next(1,:) = position_vertices(facets(facets_attached(i),1),:) - point_current;
    vectors_edge_point_next(2,:) = position_vertices(facets(facets_attached(i),2),:) - point_current;
    vectors_edge_point_next(3,:) = position_vertices(facets(facets_attached(i),3),:) - point_current;
    % 去除与新路径点重合的顶点产生的边向量
    vectors_edge_point_next(all(vectors_edge_point_next == 0,2),:) = [];
    % 将当前面片的顶点角加入总顶点角中
    angle_sum = angle_sum + getAngle(vectors_edge_point_next(1,:),vectors_edge_point_next(2,:));
end
phi = angle_sum/2;% 均分角

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 寻找新方向向量所在的三角面片 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 任选当前三角面片的一个其余顶点，与新路径点组成相邻边
index_vertice_neighbor = facets(index_facet_current,:);% 当前三角面片的所有顶点序号
index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];% 删除新路径点的序号
index_vertice_neighbor = index_vertice_neighbor(1);% 任选当前三角面片的一个其余顶点

vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);% 相邻边线的向量
phi_sum = getAngle(direction_vector_W,vector_edge_neighbor);% 计算当前相邻边与方向向量的夹角

index_last_facet_neighbor = index_facet_current;% 保存当前三角面片的序号

% 循环
while phi_sum < phi % 判断累加夹角是否超出均分角
    % 新相邻三角面片
    index_facet_neighbor = edgeAttachments(TR,index_point_current,index_vertice_neighbor);% 相邻边所在的两个三角面片的序号
    index_facet_neighbor = index_facet_neighbor{1,1};
    index_facet_neighbor(index_facet_neighbor == index_last_facet_neighbor) = [];% 删除上一个三角面片的序号

    if isempty(index_facet_neighbor)
        flag_empty = 1;
        break
    else
        % 新相邻顶点
        index_last_vertice_neighbor = index_vertice_neighbor;% 保存上一个相邻顶点
        index_vertice_neighbor = facets(index_facet_neighbor,:);% 当前三角面片的所有顶点序号
        index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];% 删除新路径点的序号
        index_vertice_neighbor(index_vertice_neighbor == index_last_vertice_neighbor) = [];% 删除上一个相邻顶点的序号
        % 新相邻边
        vector_last_edge_neighbor = vector_edge_neighbor;% 保存上一个相邻边向量
        vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);% 计算当前相邻边
        phi_sum = phi_sum + getAngle(vector_last_edge_neighbor,vector_edge_neighbor);% 计算当前相邻边与上一个相邻边的夹角
    end
    index_last_facet_neighbor = index_facet_neighbor;% 保存当前三角面片的序号
end

%%%%%%%%%%%%%%%%%%%%%
%%% 确定新方向向量 %%%
%%%%%%%%%%%%%%%%%%%%%
if exist('index_facet_neighbor',"var") % 判断累加夹角是否在第一次循环便超出均分角
    if isempty(index_facet_neighbor)
        T = [];
    else
        alpha = phi_sum - phi;% 将均分角与累加夹角之差作为新的方向角
        index_facet_current = index_facet_neighbor;% 将累加夹角超出均分角时的相邻面片序号作为新三角面片的序号

        % 建立新的三角面片局部坐标系
        z_basis = faceNormal(TR,index_facet_current);
        z_basis = z_basis/norm(z_basis);
        x_basis = -vector_edge_neighbor;% 将上一个相邻边向量的反方向作为x基向量的方向
        x_basis = x_basis/norm(x_basis);
        y_basis = cross(z_basis,x_basis);
        T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

        % 建立方向向量
        direction_vector_F = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
        direction_vector_W = direction_vector_W(1:3);

        % 保证方向向量向前
        if dot(-vector_last_edge_neighbor,y_basis) < 0
            alpha = -alpha;
            % 建立方向向量
            direction_vector_F = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
            direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
            direction_vector_W = direction_vector_W(1:3);
        end
    end
else
    alpha = phi_sum - phi;% 将均分角与累加夹角之差作为新的方向角
    % 建立新的三角面片局部坐标系
    z_basis = faceNormal(TR,index_facet_current);
    z_basis = z_basis/norm(z_basis);
    x_basis = -vector_edge_neighbor;% 将上一个相邻边向量的反方向作为x基向量的方向
    x_basis = x_basis/norm(x_basis);
    y_basis = cross(z_basis,x_basis);
    T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

    % 建立方向向量
    direction_vector_F = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
    direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
    direction_vector_W = direction_vector_W(1:3);

    % 保证方向向量向前
    if dot(-vector_edge_neighbor,y_basis) < 0
        alpha = -alpha;
        % 建立方向向量
        direction_vector_F = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
        direction_vector_W = direction_vector_W(1:3);
    end
end
end