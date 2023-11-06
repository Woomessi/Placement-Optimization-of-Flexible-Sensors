clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

% 连杆三角面片模型输入
TR = stlread('C:\projects\MATLAB\robot_sensor\meshes\stl_refined\link0.stl');
X = TR.Points;
F = TR.ConnectivityList;
phi = 60;% 方向角

% 选择起点
index_F_current = 1;
P_current = incenter(TR,index_F_current);

% 初始化路径点序列
P_seq = P_current;
F_seq = index_F_current;
round = 1; % 路径点编号
vertex1 = X(F(index_F_current,1),:);
vertex2 = X(F(index_F_current,2),:);
vertex3 = X(F(index_F_current,3),:);

% 建立三角面片局部坐标系
z_basis = faceNormal(TR,index_F_current);
z_basis = z_basis/norm(z_basis);
x_basis = vertex1 - P_current;
x_basis = x_basis/norm(x_basis);
y_basis = cross(z_basis,x_basis);
R = [x_basis;y_basis;z_basis]';% 三角面片局部坐标系相对于世界坐标系的旋转变换矩阵

t_T = [cosd(phi),sind(phi),0];% 三角面片局部坐标系中的前进方向
t_W = (R*t_T')';% 世界坐标系中的前进方向

%%%%%%%%%%%%%%%%
%%% 计算交点 %%%
%%%%%%%%%%%%%%%%

% 计算由当前点指向三角面片各顶点的"顶点向量"
vector1 = vertex1 - P_current;
vector2 = vertex2 - P_current;
vector3 = vertex3 - P_current;

% 计算各顶点向量的夹角
ang12 = getAngle(vector1,vector2);
ang13 = getAngle(vector1,vector3);
ang23 = getAngle(vector2,vector3);

% 计算前进方向与各顶点向量的夹角
ang_t_1 = getAngle(vector1,t_W);
ang_t_2 = getAngle(vector2,t_W);
ang_t_3 = getAngle(vector3,t_W);

% 计算交点
if ang12 == ang_t_1 + ang_t_2
    P_next = getIntersection(P_current,t_W,vertex1,vertex2 - vertex1);
else
    if ang13 == ang_t_1 + ang_t_3
        P_next = getIntersection(P_current,t_W,vertex1,vertex3 - vertex1);
    else
        P_next = getIntersection(P_current,t_W,vertex2,vertex3 - vertex2);
    end
end

if P_next == vertex1
    facet_attached = vertexAttachments(TR,F(index_F_current,1));
    facet_attached = facet_attached{1,1};
    vectors_angle = zeros(3,3);
    ang_sum = 0;
    for i = 1:size(facet_attached,2)
        vectors_angle(1,:) = X(F(facet_attached(i),1),:) - P_next;
        vectors_angle(2,:) = X(F(facet_attached(i),2),:) - P_next;
        vectors_angle(3,:) = X(F(facet_attached(i),3),:) - P_next;
        vectors_angle(all(vectors_angle==0,2),:)=[];
        ang_sum = ang_sum + getAngle(vectors_angle(1,:),vectors_angle(2,:));
    end
    phi = ang_sum/2;
else
    if P_next == vertex2
        facet_attached = vertexAttachments(TR,F(index_F_current,2));
        facet_attached = facet_attached{1,1};
        vectors_angle = zeros(3,3);
        ang_sum = 0;
        for i = 1:size(facet_attached,2)
            vectors_angle(1,:) = X(F(facet_attached(i),1),:) - P_next;
            vectors_angle(2,:) = X(F(facet_attached(i),2),:) - P_next;
            vectors_angle(3,:) = X(F(facet_attached(i),3),:) - P_next;
            vectors_angle(all(vectors_angle==0,2),:)=[];
            ang_sum = ang_sum + getAngle(vectors_angle(1,:),vectors_angle(2,:));
        end
        phi = ang_sum/2;
    else
        if P_next == vertex3
            facet_attached = vertexAttachments(TR,F(index_F_current,3));
            facet_attached = facet_attached{1,1};
            vectors_angle = zeros(3,3);
            ang_sum = 0;
            for i = 1:size(facet_attached,2)
                vectors_angle(1,:) = X(F(facet_attached(i),1),:) - P_next;
                vectors_angle(2,:) = X(F(facet_attached(i),2),:) - P_next;
                vectors_angle(3,:) = X(F(facet_attached(i),3),:) - P_next;
                vectors_angle(all(vectors_angle==0,2),:)=[];
                ang_sum = ang_sum + getAngle(vectors_angle(1,:),vectors_angle(2,:));
            end
            phi = ang_sum/2;
        else
        end
    end
end

% 更新
% round = round + 1;% 路径点编号
% P_current = P_next;
% P_seq(round,:) = P_current;
% F_seq(round) = index_F_current;

% %{
% 作图
F_color = zeros(size(F,1),1);
F_color(index_F_current) = 1;
patch('Faces',F,'Vertices',X,'FaceVertexCData',F_color,'FaceColor','flat');
% patch('Faces',F(F_0,:),'Vertices',X,'FaceColor','green');
colormap hot
colorbar
axis equal
hold on

scatter3(P_current(1),P_current(2),P_current(3),18,"magenta",'filled')
scatter3(P_next(1),P_next(2),P_next(3),18,"magenta",'filled')

quiver3(vertex1(1),vertex1(2),vertex1(3),x_basis(1),x_basis(2),x_basis(3),0.1,"filled",'r','LineWidth',0.1);
quiver3(vertex1(1),vertex1(2),vertex1(3),y_basis(1),y_basis(2),y_basis(3),0.1,"filled",'g','LineWidth',0.1);
quiver3(vertex1(1),vertex1(2),vertex1(3),z_basis(1),z_basis(2),z_basis(3),0.1,"filled",'b','LineWidth',0.1);

quiver3(P_current(1),P_current(2),P_current(3),t_W(1),t_W(2),t_W(3),0.1,"filled",'m','LineWidth',0.1);
%}