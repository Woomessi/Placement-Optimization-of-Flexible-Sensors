function [direction_vector_W, point_current, T] = getEdgePathpoint(TR, vertex_opposite, index_facet_current, point_next, x_basis, direction_vector_W)

point_current = point_next;% Update the current path point

% Establish a new local frame
z_basis = faceNormal(TR,index_facet_current);
z_basis = z_basis/norm(z_basis);
y_basis = cross(z_basis,x_basis);
T = [x_basis',y_basis',z_basis',point_current';0,0,0,1];

% Ensure a forward direction vector
vector_vertex_opposite = vertex_opposite - point_current;

alpha = getAngle(direction_vector_W,x_basis);
direction_vector_F1 = [cos(alpha),sin(alpha),0,1];
direction_vector_W1 = (T*direction_vector_F1')'-(T*[zeros(3,1);1])';
direction_vector_W1 = direction_vector_W1(1:3);

alpha = -alpha;

direction_vector_F2 = [cos(alpha),sin(alpha),0,1];
direction_vector_W2 = (T*direction_vector_F2')'-(T*[zeros(3,1);1])';
direction_vector_W2 = direction_vector_W2(1:3);


if getAngle(vector_vertex_opposite,direction_vector_W1) < getAngle(vector_vertex_opposite,direction_vector_W2)
    direction_vector_W = direction_vector_W1;
else
    direction_vector_W = direction_vector_W2;
end
end