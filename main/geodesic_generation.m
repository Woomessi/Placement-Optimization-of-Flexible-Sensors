clear
clc
close all
clear options;
addpath(genpath('C:\Placement-Optimization-of-Flexible-Sensors-release'))

%%%%%%%%%%%%%%%%%%%%%%
%%% Initialization %%%
%%%%%%%%%%%%%%%%%%%%%%

% Input the STL model of the link
TR = stlread('C:\Placement-Optimization-of-Flexible-Sensors-release\franka_description\meshes\visual\link1.stl');
position_vertices = TR.Points;% Vertex position
connectivity_facets = TR.ConnectivityList;% Facet connectivity
size_facets = size(connectivity_facets,1);

% Read the geometric information of the STL model
N_v = vertexNormal(TR); % Compute the vertex normal
N_f = faceNormal(TR); % Compute the facet normal
N_refined = [N_v;N_f];

C = incenter(TR); % Compute the triangle's centroid
X_refined = [position_vertices;C];

length_desired = 0.27;% Length of the sensor
interval_angle = 30;% Interval of the direction vector
size_angle = 360/interval_angle;

all_points_seq = cell(1,size_facets*size_angle);
index_geodesic = 1;
for index_facet = 1:size_facets% Traverse the initial point
    index_facet_start = index_facet;% Initialize the index of the current triangle
    point_start = incenter(TR,index_facet_start);% Initialize the current point
    for index_angle = 1:size_angle% Traverse the initial angle
        flag = 0;% Clear the flag of high curvature
        flag_empty = 0; % Clear the flag of empty edge
        index_facet_current = index_facet_start;
        point_current = point_start;
        % Choose the direction angle (degree)
        alpha = interval_angle*index_angle;
        % Initialize the path point sequence
        index_vertices = zeros(1,2);% Initialize the path point indices
        points_seq = point_current;% Path point sequence
        indices_facets_seq = index_facet_current;% Triangle index sequence

        % Establish the local frame of a triangle
        z_basis = faceNormal(TR,index_facet_current);
        z_basis = z_basis/norm(z_basis);% z-basis
        vertex1 = position_vertices(connectivity_facets(index_facet_current,1),:);
        x_basis = vertex1 - point_current;
        x_basis = x_basis/norm(x_basis);% x-basis
        y_basis = cross(z_basis,x_basis);% y-basis
        T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];% Transformation matrix from the local frame to the global one

        % Establish the direction vector
        direction_vector_F = [cosd(alpha),sind(alpha),0,1];% Direction vector in the local frame
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';% Direction vector in the global frame
        direction_vector_W = direction_vector_W(1:3);

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Update the path point %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        round = 2;
        length_sum = 0;
        while length_sum < length_desired

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% 1.1 Is the current path point a triangle vertex? %%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            if size(index_vertices,2) == 1
                % Extract the current triangle's vertices apart from the current path point
                indices_vertices = connectivity_facets(index_facet_current,:);
                indices_vertices(indices_vertices == index_vertices) = [];
                vertex1 = position_vertices(indices_vertices(1),:);% The first vertex of the current triangle
                vertex2 = position_vertices(indices_vertices(2),:);% The second vertex of the current triangle

                % Compute the intersection point between the direction
                % vector and the edge between vertex1 and vertex2 from the
                % current path point
                point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex2 - vertex1);
                if isempty(point_next)
                    flag = 1;
                    break
                end

                x_basis = (vertex1 - point_next)/norm(vertex1 - point_next);% Establish a new x-basis on the intersection edge
                index_vertices = indices_vertices;% Indices of vertex1 and vertex2

                % Update the current triangle's index
                index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% The indices of the attached triangles of the intersection edge
                index_facets_attached = index_facets_attached{1,1};
                index_facets_attached(index_facets_attached == index_facet_current) = [];% Delete the current triangle's index
                index_facet_current = index_facets_attached;% Update the current triangle's index
                % Compute the opposite vertex
                if isempty(index_facet_current)
                    flag_empty = 1;
                    break
                else
                    index_vertex_opposite = connectivity_facets(index_facet_current,:);
                    index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                    index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                    vertex_opposite = position_vertices(index_vertex_opposite,:);
                    [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                end
            else

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% 1.2 Otherwise, the current path point is not a vertex%%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                vertex1 = position_vertices(connectivity_facets(index_facet_current,1),:);% 当前三角面片的第1个顶点
                vertex2 = position_vertices(connectivity_facets(index_facet_current,2),:);% 当前三角面片的第2个顶点
                vertex3 = position_vertices(connectivity_facets(index_facet_current,3),:);% 当前三角面片的第3个顶点

                % Compute the vectors from the current path point to the vertices of the current triangle
                vector_vertex1 = vertex1 - point_current;
                vector_vertex1 = vector_vertex1/norm(vector_vertex1);
                vector_vertex2 = vertex2 - point_current;
                vector_vertex2 = vector_vertex2/norm(vector_vertex2);
                vector_vertex3 = vertex3 - point_current;
                vector_vertex3 = vector_vertex3/norm(vector_vertex3);

                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %%% 2.1 Does the direction vector meet a vertex of the current triangle %%%
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                % Is the direction vector along vector_vertex1 
                if abs(vector_vertex1 - direction_vector_W) < ones(1,3)*0.01
                    point_current = vertex1;% Select vertex1 as the new path point
                    index_vertices = connectivity_facets(index_facet_current,1);% The index of the current path point

                    [direction_vector_W,index_facet_current,point_current,T,flag_empty] = getVertexPathpoint_1021(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                    if flag_empty == 1
                        break
                    end
                else
                    % Is the direction vector along vector_vertex2
                    if abs(vector_vertex2 - direction_vector_W) < ones(1,3)*0.01
                        point_current = vertex2;
                        index_vertices = connectivity_facets(index_facet_current,2);
                        [direction_vector_W,index_facet_current,point_current,T,flag_empty] = getVertexPathpoint_1021(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                        if flag_empty == 1
                            break
                        end
                    else
                        % Is the direction vector along vector_vertex3
                        if abs(vector_vertex3 - direction_vector_W) < ones(1,3)*0.01
                            point_current = vertex3;
                            index_vertices = connectivity_facets(index_facet_current,3);
                            [direction_vector_W,index_facet_current,point_current,T,flag_empty] = getVertexPathpoint_1021(TR, index_vertices, position_vertices, connectivity_facets, point_current, index_facet_current, direction_vector_W);
                            if flag_empty == 1
                                break
                            end
                        else

                            %%%%%%%%%%%%%%%%%%%%%
                            %%% 2.2 Otherwise %%%
                            %%%%%%%%%%%%%%%%%%%%%

                            angle_12 = getAngle(vector_vertex1,vector_vertex2);
                            angle_13 = getAngle(vector_vertex1,vector_vertex3);
                            angle_23 = getAngle(vector_vertex2,vector_vertex3);

                            angle_direction_vector_1 = getAngle(vector_vertex1,direction_vector_W);
                            angle_direction_vector_2 = getAngle(vector_vertex2,direction_vector_W);
                            angle_direction_vector_3 = getAngle(vector_vertex3,direction_vector_W);

                            if abs(angle_12 - (angle_direction_vector_1 + angle_direction_vector_2)) <= 0.01 && abs(angle_12 - pi) >= 0.01
                                point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex2 - vertex1);
                                if isempty(point_next)
                                    flag = 1;
                                    break
                                end
                                x_basis = (vertex1 - point_next)/norm(vertex1 - point_next);% 在相交边线上建立新的x轴基向量
                                index_vertices = [connectivity_facets(index_facet_current,1),connectivity_facets(index_facet_current,2)];% 顶点1与顶点2的序号

                                index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                                index_facets_attached = index_facets_attached{1,1};
                                index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                                index_facet_current = index_facets_attached;

                                if isempty(index_facet_current)
                                    flag_empty = 1;
                                    break
                                else
                                    index_vertex_opposite = connectivity_facets(index_facet_current,:);
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                                    index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                                    vertex_opposite = position_vertices(index_vertex_opposite,:);
                                    [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                                end
                            else

                                if abs(angle_13 - (angle_direction_vector_1 + angle_direction_vector_3)) <= 0.01 && abs(angle_13 - pi) >= 0.01
                                    point_next = getIntersection(point_current,direction_vector_W,vertex1,vertex3 - vertex1);
                                    if isempty(point_next)
                                        flag = 1;
                                        break
                                    end
                                    x_basis = (vertex3 - point_next)/norm(vertex3 - point_next);
                                    index_vertices = [connectivity_facets(index_facet_current,1),connectivity_facets(index_facet_current,3)];

                                    index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));% 与交点所在边线相连接的两个三角面片的序号
                                    index_facets_attached = index_facets_attached{1,1};
                                    index_facets_attached(index_facets_attached == index_facet_current) = [];% 删除当前三角面片序号
                                    index_facet_current = index_facets_attached;
                                    if isempty(index_facet_current)
                                        flag_empty = 1;
                                        break
                                    else
                                        index_vertex_opposite = connectivity_facets(index_facet_current,:);
                                        index_vertex_opposite(index_vertex_opposite == index_vertices(1)) = [];
                                        index_vertex_opposite(index_vertex_opposite == index_vertices(2)) = [];
                                        vertex_opposite = position_vertices(index_vertex_opposite,:);
                                        [direction_vector_W,point_current,T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W);
                                    end
                                else

                                    point_next = getIntersection(point_current,direction_vector_W,vertex2,vertex3 - vertex2);
                                    if isempty(point_next)
                                        flag = 1;
                                        break
                                    end
                                    x_basis = (vertex2 - point_next)/norm(vertex2 - point_next);
                                    index_vertices = [connectivity_facets(index_facet_current,2),connectivity_facets(index_facet_current,3)];

                                    index_facets_attached = edgeAttachments(TR,index_vertices(1),index_vertices(2));
                                    index_facets_attached = index_facets_attached{1,1};
                                    index_facets_attached(index_facets_attached == index_facet_current) = [];
                                    index_facet_current = index_facets_attached;

                                    if isempty(index_facet_current)
                                        flag_empty = 1;
                                        break
                                    else
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
            end

            points_seq(round,:) = point_current;
            indices_facets_seq(round) = index_facet_current;
            if round >1
                if abs(points_seq(round,1)-points_seq(round-1,1))<0.00001 && abs(points_seq(round,2)-points_seq(round-1,2))<0.00001 && abs(points_seq(round,3)-points_seq(round-1,3))<0.00001
                    flag = 1;
                    break
                end
            end

            if round == 1
                length = 0;
            else
                length = norm(points_seq(round,:)-points_seq(round-1,:));
            end

            round = round + 1;
            length_sum = length_sum + length;
        end

        if flag == 0 && flag_empty == 0
            t = 1-sqrt((length_desired-length_sum+length).^2*((points_seq(round-2,1)-points_seq(round-1,1)).^2+(points_seq(round-2,2)-points_seq(round-1,2)).^2+(points_seq(round-2,3)-points_seq(round-1,3)).^2))./((points_seq(round-2,1)-points_seq(round-1,1)).^2+(points_seq(round-2,2)-points_seq(round-1,2)).^2+(points_seq(round-2,3)-points_seq(round-1,3)).^2);
            points_seq(round-1,:) = t*points_seq(round-2,:)+(1-t)*points_seq(round-1,:);
            all_points_seq{1,index_geodesic} = points_seq';
            index_geodesic = index_geodesic + 1;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%
%%% Postprocessing %%%
%%%%%%%%%%%%%%%%%%%%%%

size_sensor = 16;
interval_segment = length_desired/(size_sensor-1);
size_all_points_seq = index_geodesic - 1;
all_points_seq = all_points_seq(1,1:size_all_points_seq);
all_tform_spot = cell(1,size_all_points_seq);

all_spot = cell(1,size_all_points_seq);
all_first_spot = zeros(3,size_all_points_seq);
all_middle_spot = zeros(3,size_all_points_seq);
all_last_spot = zeros(3,size_all_points_seq);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Extract the pose of each ToF module %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for idx_geodesic = 1:size_all_points_seq

    points_seq_current = all_points_seq{1,idx_geodesic};
    size_points_geodesic = size(points_seq_current,2);

    length_segment = 0;
    length_segment_sum = 0;
    flag = 0;
    spot = zeros(3,size_sensor);
    y_point_path = zeros(3,size_sensor);

    spot(:,1) = points_seq_current(:,1);
    y_point_path(:,1) = points_seq_current(:,2) - points_seq_current(:,1);
    y_point_path(:,1) = y_point_path(:,1)./norm(y_point_path(:,1));

    spot(:,size_sensor) = points_seq_current(:,size_points_geodesic);
    y_point_path(:,size_sensor) = points_seq_current(:,size_points_geodesic) - points_seq_current(:,size_points_geodesic-1);
    y_point_path(:,size_sensor) = y_point_path(:,size_sensor)./norm(y_point_path(:,size_sensor));

    i = 1;
    j = 2;
    while spot(:,size_sensor-1) == zeros(3,1)
        if flag == 0
            p2 = points_seq_current(:,i+1);
            p1 = points_seq_current(:,i);
        else
            p2 = points_seq_current(:,i+1);
            p1 = spot(:,j-1);
            flag = 0;
        end
        vector1 = p2 - p1;
        length_segment = length_segment + norm(vector1);

        if length_segment > interval_segment
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

    ID_all = dsearchn(X_refined,spot');

    x_point_path = N_refined(ID_all,:)';

    z_point_path = cross(x_point_path,y_point_path);
    z_point_path = z_point_path./vecnorm(z_point_path);

    y_point_path = cross(z_point_path,x_point_path);

    tform_spot_current = cell(1,size_sensor);
    for i = 1:size_sensor
        tform_spot_current{1,i} = [x_point_path(:,i),y_point_path(:,i),z_point_path(:,i),spot(:,i);0,0,0,1];
    end
    all_tform_spot{1,idx_geodesic} = tform_spot_current;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Exclude the repeated geodesics %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

repeated_geodesic = zeros(1,size_all_points_seq);
idx_geodesic = 1:size_all_points_seq;
tol = 0.005;
for i = 1:size_all_points_seq-1
    for j = i+1:size_all_points_seq
        difference_first_spot = abs(all_first_spot(:,idx_geodesic(i)) - all_first_spot(:,idx_geodesic(j)));
        difference_middle_spot = abs(all_middle_spot(:,idx_geodesic(i)) - all_middle_spot(:,idx_geodesic(j)));
        difference_last_spot = abs(all_last_spot(:,idx_geodesic(i)) - all_last_spot(:,idx_geodesic(j)));
        if difference_first_spot(1) < tol && difference_first_spot(2) < tol && difference_first_spot(3) < tol
            if difference_middle_spot(1) < tol && difference_middle_spot(2) < tol && difference_middle_spot(3) < tol
                if difference_last_spot(1) < tol && difference_last_spot(2) < tol && difference_last_spot(3) < tol
                    idx_geodesic(i) = 0;
                    break
                end
            end
        end
    end
end
idx_geodesic(:,all(idx_geodesic==0,1))=[];
all_tform_spot = all_tform_spot(1,idx_geodesic);
