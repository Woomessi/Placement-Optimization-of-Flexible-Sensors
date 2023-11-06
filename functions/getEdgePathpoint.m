function [direction_vector_W, point_current, T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W)
% 搜寻下一个三角面片
% index_facets_attached = edgeAttachments(TR,index_vertex(1),index_vertex(2));% 与交点所在边线相连接的两个三角面片的序号
% index_facets_attached = index_facets_attached{1,1};
% index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号

point_current = point_next;% 更新当前路径点
% points_seq = [points_seq;point_current];
% indices_facets_seq = [indices_facets_seq,index_facet_current];

% 建立新的局部坐标系
z_basis = faceNormal(TR,index_facet_current);
z_basis = z_basis/norm(z_basis);
y_basis = cross(z_basis,x_basis);
T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

% 保证方向向量向前
vector_vertex_opposite = vertex_opposite - point_current;

% quiver3(point_current(1),point_current(2),point_current(3),vector_vertex_opposite(1),vector_vertex_opposite(2),vector_vertex_opposite(3),0.001,"filled",'r','LineWidth',0.1);


alpha = getAngle(direction_vector_W,x_basis);
direction_vector_F1 = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
direction_vector_W1 = (T*direction_vector_F1')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
direction_vector_W1 = direction_vector_W1(1:3);

% quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W1(1),direction_vector_W1(2),direction_vector_W1(3),0.001,"filled",'r','LineWidth',0.1);

alpha = -alpha;
% 建立方向向量
direction_vector_F2 = [cos(alpha),sin(alpha),0,1];% 三角面片局部坐标系中的方向向量。起点为原点时，该变量表示终点
direction_vector_W2 = (T*direction_vector_F2')'-(T*[zeros(3,1);1])';% 世界坐标系中的方向向量，（终点 - 起点）方为矢量，需进行两次对点坐标的齐次变换
direction_vector_W2 = direction_vector_W2(1:3);

% quiver3(point_current(1),point_current(2),point_current(3),direction_vector_W2(1),direction_vector_W2(2),direction_vector_W2(3),0.001,"filled",'r','LineWidth',0.1);


if getAngle(vector_vertex_opposite,direction_vector_W1) < getAngle(vector_vertex_opposite,direction_vector_W2)
    direction_vector_W = direction_vector_W1;
else
    direction_vector_W = direction_vector_W2;
end
end