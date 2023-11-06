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

size_facets = size(connectivity_facets,1);
interval_angle = 30;% 角度间隔
size_angle = 360/interval_angle;

length_desired = 0.27;% 传感器长度

all_points_seq = cell(1,size_facets*size_angle);

flag = 0;% 是否遭遇曲率突变的标志
index_geodesic = 1;

% for index_facet = 1:size_facets
%     % 选择起点
%     index_facet_current = index_facet;% 初始化当前三角面片索引
%     point_current = incenter(TR,index_facet_current);% 初始化当前路径点
%     for index_angle = 1:size_angle
%         % 选择方向角（角度制）
%         alpha = interval_angle*index_angle;

        index_facet = 1;
        index_angle = 1;
        index_facet_current = index_facet;% 初始化当前三角面片索引
        point_current = incenter(TR,index_facet_current);% 初始化当前路径点
        alpha = interval_angle*index_angle;

        % 初始化路径点序列
        index_vertices = zeros(1,2);% 初始化路径点序号
        points_seq = point_current;% 路径点序列
        indices_facets_seq = index_facet_current;% 三角面片索引序列

        % 建立三角面片局部坐标系
        z_basis = faceNormal(TR,index_facet_current);
        z_basis = z_basis/norm(z_basis);% z基向量
        vertex1 = position_vertices(connectivity_facets(index_facet_current,1),:);% 当前三角面片的第1个顶点
        x_basis = vertex1 - point_current;
        x_basis = x_basis/norm(x_basis);% x基向量
        y_basis = cross(z_basis,x_basis);% y基向量
        T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

        % 建立方向向量
        direction_vector_F = [cosd(alpha),sind(alpha),0,1];% 方向向量（三角面片局部坐标系）。起点为原点时，该变量表示终点
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% 方向向量（世界坐标系）。[终点 - 起点]方为矢量，故需同时对终点和起点坐标进行齐次变换
        direction_vector_W = direction_vector_W(1:3);

        %%%%%%%%%%%%%%%%%
        %%% 更新路径点 %%%
        %%%%%%%%%%%%%%%%%

        round = 1;
        length_sum = 0;
        while length_sum < length_desired

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% 1.1 判断当前路径点是否为三角面片顶点 %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if size(index_vertices,2) == 1 % 路径点为三角面片顶点（index_vertices元素数量为1）
                % 提取当前三角面片除当前路径点外其它顶点
                indices_vertices = connectivity_facets(index_facet_current,:);
                indices_vertices(indices_vertices == index_vertices) = [];
                vertex1 = position_vertices(indices_vertices(1),:);% 当前三角面片的第1个顶点
                vertex2 = position_vertices(indices_vertices(2),:);% 当前三角面片的第2个顶点

                % 从当前路径点（顶点）出发，计算方向向量与其余两顶点所夹边的交点
                point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex2 - vertex1);

                x_basis = (vertex1 - point_next)/norm(vertex1 - point_next);% 在相交边线上建立新的x轴基向量
                index_vertices = indices_vertices;% 顶点1与顶点2的序号

                % 更新当前三角面片序号
                index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                index_facets_attached = index_facets_attached{1,1};
                index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                index_facet_current = index_facets_attached;% 更新当前三角面片序号

                % 计算对顶点
                index_vertex_opposite = connectivity_facets(index_facet_current,:);
                index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                vertex_opposite = position_vertices(index_vertex_opposite,:);

                [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
            else

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% 1.2 否则，当前路径点不是顶点 %%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% 2.1 判断方向向量是否与当前三角面片相交于某个顶点 %%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % 判断方向向量是否同顶点向量1重合
                if abs(vector_vertex1 - direction_vector_W) < ones(1,3)*0.01
                    point_current = vertex1;% 将顶点1作为新的当前路径点
                    index_vertices = connectivity_facets(index_facet_current,1);% 当前路径点的索引

                    [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint_new(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                else
                    % 判断方向向量是否同顶点向量2重合
                    if abs(vector_vertex2 - direction_vector_W) < ones(1,3)*0.01
                        point_current = vertex2;
                        index_vertices = connectivity_facets(index_facet_current,2);

                        [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint_new(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                    else
                        % 判断方向向量是否同顶点向量3重合
                        if abs(vector_vertex3 - direction_vector_W) < ones(1,3)*0.01
                            point_current = vertex3;
                            index_vertices = connectivity_facets(index_facet_current,3);

                            [direction_vector_W,index_facet_current,point_current,T] = getVertexPathpoint_new(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                        else

                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            %%% 2.2 否则，方向向量与当前三角面片边线中间的点相交 %%%
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

                                % 更新当前三角面片序号
                                index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                                index_facets_attached = index_facets_attached{1,1};
                                index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                                index_facet_current = index_facets_attached;

                                % 计算对顶点
                                index_vertex_opposite = connectivity_facets(index_facet_current,:);
                                index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                                index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                                vertex_opposite = position_vertices(index_vertex_opposite,:);

                                [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                            else

                                % 方向向量相交于顶点1与顶点3之间的边线，将交点作为新的路径点
                                if abs(angle_13 - (angle_direction_vector_1 + angle_direction_vector_3)) <= 0.01 && abs(angle_13 - pi) >= 0.01
                                    point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex3 - vertex1);
                                    x_basis = (vertex3 - point_next)/norm(vertex3 - point_next);
                                    index_vertices = [connectivity_facets(index_facet_current,1),connectivity_facets(index_facet_current,3)];

                                    % 更新当前三角面片序号
                                    index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                                    index_facets_attached = index_facets_attached{1,1};
                                    index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                                    index_facet_current = index_facets_attached;

                                    % 计算对顶点
                                    index_vertex_opposite = connectivity_facets(index_facet_current,:);
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                                    vertex_opposite = position_vertices(index_vertex_opposite,:);

                                    [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                                else

                                    % 方向向量相交于顶点2与顶点3之间的边线，将交点作为新的路径点
                                    point_next = getIntersection(point_current,direction_vector_W,vertex2,vertex3 - vertex2);
                                    x_basis = (vertex2 - point_next)/norm(vertex2 - point_next);
                                    index_vertices = [connectivity_facets(index_facet_current,2),connectivity_facets(index_facet_current,3)];

                                    % 更新当前三角面片序号
                                    index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                                    index_facets_attached = index_facets_attached{1,1};
                                    index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                                    index_facet_current = index_facets_attached;

                                    % 计算对顶点
                                    index_vertex_opposite = connectivity_facets(index_facet_current,:);
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                                    vertex_opposite = position_vertices(index_vertex_opposite,:);

                                    [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                                end
                            end
                        end
                    end
                end
            end

            % 更新路径信息
            points_seq(round,:) = point_current;
            indices_facets_seq(round) = index_facet_current;

            % 排除曲率突变
            if round >1
                if dot(faceNormal(TR,indices_facets_seq(round)),faceNormal(TR,indices_facets_seq(round-1)))<=0.05
                    flag = 1;
                    break
                end
            end

            % 计算长度
            if round == 1
                length = 0;
            else
                length = norm(points_seq(round,:)-points_seq(round-1,:));
            end

            round = round + 1;
            length_sum = length_sum + length;

        end

        % 调整最后一个路径点的位置，使其满足传感器长度条件
        if flag == 0
            t = 1-sqrt((length_desired-length_sum+length).^2*((points_seq(round-2,1)-points_seq(round-1,1)).^2+(points_seq(round-2,2)-points_seq(round-1,2)).^2+(points_seq(round-2,3)-points_seq(round-1,3)).^2))./((points_seq(round-2,1)-points_seq(round-1,1)).^2+(points_seq(round-2,2)-points_seq(round-1,2)).^2+(points_seq(round-2,3)-points_seq(round-1,3)).^2);
            points_seq(round-1,:) = t*points_seq(round-2,:)+(1-t)*points_seq(round-1,:);
            all_points_seq{1,index_geodesic} = points_seq';
            index_geodesic = index_geodesic + 1;
        end
%     end
% end
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

% patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','white','FaceAlpha',.8);
patch('Faces',connectivity_facets,'Vertices',position_vertices,'FaceColor','white','EdgeColor','none','FaceAlpha',.8);

scatter3(points_seq(:,1),points_seq(:,2),points_seq(:,3),18,"magenta",'filled')