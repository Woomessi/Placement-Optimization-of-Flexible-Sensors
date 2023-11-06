function [phi,paths] = heatMethod(X,F,pstart,pend)

n = size(X,2); % 顶点数量
m = size(F,2); % 三角面数量

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 计算三角面片其它信息 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
XF = @(i)X(:,F(i,:)); % 提取三角面片中序号为i的顶点的坐标
amplitude = @(X)sqrt( sum( X.^2 ) );
normalize = @(X)X ./ repmat(amplitude(X), [3 1]);

Na = cross( XF(2)-XF(1), XF(3)-XF(1) ); % 三角面片法向量
N = normalize(Na); % 三角面片单位法向量
A = amplitude(Na)/2; % 每个三角面片的面积

%%%%%%%%%%%%%%%%%%%%%%%%
%%% 将stl转换为graph %%%
%%%%%%%%%%%%%%%%%%%%%%%%
v1 = [F(1,:)';F(1,:)';F(2,:)'];
v2 = [F(2,:)';F(3,:)';F(3,:)'];

G = graph(v1,v2);
G = simplify(G); % 去除重复边

% 作图
% plot(G,'XData',X(1,:),'YData',X(2,:),'ZData',X(3,:))
% axis equal
% hold on
% scatter3(X(1,pstart),X(2,pstart),X(3,pstart),'m','filled');
% scatter3(X(1,pend), X(1,pend), X(1,pend),'y','filled');

%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 计算三角面片平均边长 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
size_edge = size(G.Edges,1); % 三角面片边线数量
d_all = 0;
Edges = table2cell(G.Edges);
for i = 1:size_edge
    d = norm(X(:,Edges{i,1}(1))-X(:,Edges{i,1}(2)));
    d_all = d_all + d;
end
d_avg = d_all/size_edge;

%%%%%%%%%%%%%%%%%%%
%%% 离散几何算子 %%%
%%%%%%%%%%%%%%%%%%%
I = []; J = []; V = []; % indexes to build the sparse matrices
for i=1:3
    % opposite edge e_i indexes
    s = mod(i,3)+1;
    t = mod(i+1,3)+1;
    % vector N_f^e_i
    wi = cross(XF(t)-XF(s),N);
    % update the index listing
    I = [I, 1:m];
    J = [J, F(i,:)];
    V = [V, wi]; % vector N_f^e_i
end

% Sparse matrix with entries 1/(2Af)
dA = spdiags(1./(2*A(:)),0,m,m);

% Compute gradient
GradMat = {};
for k=1:3
    GradMat{k} = dA*sparse(I,J,V(k,:),m,n);
end

Grad = @(u)[GradMat{1}*u, GradMat{2}*u, GradMat{3}*u]';

% Compute divergence matrices as transposed of grad for the face area inner product.
dAf = spdiags(2*A(:),0,m,m);
DivMat = {GradMat{1}'*dAf, GradMat{2}'*dAf, GradMat{3}'*dAf};

% Div operator
Div = @(q)DivMat{1}*q(1,:)' + DivMat{2}*q(2,:)' + DivMat{3}*q(3,:)';

% Laplacian operator as the composition of grad and div
Delta = DivMat{1}*GradMat{1} + DivMat{2}*GradMat{2} + DivMat{3}*GradMat{3};

% Cotan of an angle between two vectors
cota = @(a,b)cot( acos( dot(normalize(a),normalize(b)) ) );

% Compute cotan weights Laplacian
I = []; J = []; V = []; % indexes to build the sparse matrices
Ia = []; Va = []; % area of vertices
for i=1:3
    % opposite edge e_i indexes
    s = mod(i,3)+1;
    t = mod(i+1,3)+1;
    % adjacent edge
    ctheta = cota(XF(s)-XF(i), XF(t)-XF(i));
    % ctheta = max(ctheta, 1e-2); % avoid degeneracy
    % update the index listing
    I = [I, F(s,:), F(t,:)];
    J = [J, F(t,:), F(s,:)];
    V = [V, ctheta, ctheta];
    % update the diagonal with area of face around vertices
    Ia = [Ia, F(i,:)];
    Va = [Va, A];
end
% Aread diagonal matrix
Ac = sparse(Ia,Ia,Va,n,n);

% Cotan weights
Wc = sparse(I,J,V,n,n);

% Laplacian with cotan weights.
DeltaCot = spdiags(full(sum(Wc))', 0, n,n) - Wc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Heat Diffusion and Time Stepping %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t = d_avg; % 将三角面片平均长度作为 heat method 的时间步长

% Set initial condition (Dirac delta)
delta = zeros(n,1);
delta(pstart) = 1;

% Solve the linear system (single backward Euler step)
u = (Ac+t*DeltaCot)\delta;

% Compute the gradient field
g = Grad(u);

% Normalize it to obtain
h = -normalize(g);

% Integrate it back by solving
phi = Delta \ Div(h); % 当前点到其它点的测地线距离

% 作测地线距离图
% options.face_vertex_color = phi;
% plot_mesh(X,F,options);
% axis('tight');
% colormap parula(256);
% hold on
% scatter3(X(1,pstart),X(2,pstart),X(3,pstart),'m','filled');
% scatter3(X(1,pend), X(1,pend), X(1,pend),'y','filled');

%%%%%%%%%%%%%%%%%%%%%
%%% 计算测地线路径 %%%
%%%%%%%%%%%%%%%%%%%%%
options.method = 'continuous';
options.method = 'discrete';

options.face_vertex_color = phi;
paths = compute_geodesic_mesh(phi, X, F, pend, options);

%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%
% plot_fast_marching_mesh(X, F, phi, paths, options);

end