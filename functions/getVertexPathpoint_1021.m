function [direction_vector_W,index_facet_current,point_current,T,flag_empty] = getVertexPathpoint_1021(TR, index_point_current, position_vertices, facets, point_current, index_facet_current, direction_vector_W)
flag_empty = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Compute the bisecting angle %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
facets_attached = vertexAttachments(TR,index_point_current);% All the triangles contain the new path point
facets_attached = facets_attached{1,1};
vectors_edge_point_next = zeros(3,3);
angle_sum = 0;% Intialize the total vertex angle
for i = 1:size(facets_attached,2)% Traverse all the triangles contain the new path point
    % Vectors from other vertices in the current triangle to the new path point
    vectors_edge_point_next(1,:) = position_vertices(facets(facets_attached(i),1),:) - point_current;
    vectors_edge_point_next(2,:) = position_vertices(facets(facets_attached(i),2),:) - point_current;
    vectors_edge_point_next(3,:) = position_vertices(facets(facets_attached(i),3),:) - point_current;
    % Exclude the vector from the vertex that superposes with the new path point
    vectors_edge_point_next(all(vectors_edge_point_next == 0,2),:) = [];
    % Accumulate the total vertex angle with the current vertex angle
    angle_sum = angle_sum + getAngle(vectors_edge_point_next(1,:),vectors_edge_point_next(2,:));
end
phi = angle_sum/2;% bisecting angle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Search the triangle that contains the new direction vector %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

index_vertice_neighbor = facets(index_facet_current,:);
index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];
index_vertice_neighbor = index_vertice_neighbor(1);

vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);
phi_sum = getAngle(direction_vector_W,vector_edge_neighbor);

index_last_facet_neighbor = index_facet_current;

while phi_sum < phi
    index_facet_neighbor = edgeAttachments(TR,index_point_current,index_vertice_neighbor);
    index_facet_neighbor = index_facet_neighbor{1,1};
    index_facet_neighbor(index_facet_neighbor == index_last_facet_neighbor) = [];

    if isempty(index_facet_neighbor)
        flag_empty = 1;
        break
    else
        index_last_vertice_neighbor = index_vertice_neighbor;
        index_vertice_neighbor = facets(index_facet_neighbor,:);
        index_vertice_neighbor(index_vertice_neighbor == index_point_current) = [];
        index_vertice_neighbor(index_vertice_neighbor == index_last_vertice_neighbor) = [];

        vector_last_edge_neighbor = vector_edge_neighbor;
        vector_edge_neighbor = point_current - position_vertices(index_vertice_neighbor,:);
        phi_sum = phi_sum + getAngle(vector_last_edge_neighbor,vector_edge_neighbor);
    end
    index_last_facet_neighbor = index_facet_neighbor;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Find the new direction vector %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%=%%%%%%%%
if exist('index_facet_neighbor',"var")
    if isempty(index_facet_neighbor)
        T = [];
    else
        alpha = phi_sum - phi;
        index_facet_current = index_facet_neighbor;

        z_basis = faceNormal(TR,index_facet_current);
        z_basis = z_basis/norm(z_basis);
        x_basis = -vector_edge_neighbor;
        x_basis = x_basis/norm(x_basis);
        y_basis = cross(z_basis,x_basis);
        T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];

        % 建立方向向量
        direction_vector_F = [cos(alpha),sin(alpha),0,1];
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';
        direction_vector_W = direction_vector_W(1:3);

        if dot(-vector_last_edge_neighbor,y_basis) < 0
            alpha = -alpha;
            direction_vector_F = [cos(alpha),sin(alpha),0,1];
            direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';
            direction_vector_W = direction_vector_W(1:3);
        end
    end
else
    alpha = phi_sum - phi;
    z_basis = faceNormal(TR,index_facet_current);
    z_basis = z_basis/norm(z_basis);
    x_basis = -vector_edge_neighbor;
    x_basis = x_basis/norm(x_basis);
    y_basis = cross(z_basis,x_basis);
    T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];

    direction_vector_F = [cos(alpha),sin(alpha),0,1];
    direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';
    direction_vector_W = direction_vector_W(1:3);

    if dot(-vector_edge_neighbor,y_basis) < 0
        alpha = -alpha;
        direction_vector_F = [cos(alpha),sin(alpha),0,1];
        direction_vector_W = (T*direction_vector_F')'-(T*[zeros(3,1);1])';
        direction_vector_W = direction_vector_W(1:3);
    end
end
end