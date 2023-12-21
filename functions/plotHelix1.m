function plotHelix1(h_cylinder, size_edge_candidate, edge_candidate, point_all, edge, r_cylinder, point_all_cartesian)
scatter3(point_all_cartesian(:,1),point_all_cartesian(:,2),point_all_cartesian(:,3),5,'filled',"MarkerFaceColor","#757575")
axis equal
zlim([-h_cylinder 0]);
view(45,30)
grid off;
hold on
for i = 1:size_edge_candidate
    if edge_candidate(i,1) == 0 %共轭测地线满足长度条件
        idx = edge_candidate(i,2); %离散点对索引
        %起点圆柱坐标
        theta1 = point_all(edge(idx ,1),1);
        z1 = point_all(edge(idx ,1),2);
        %终点圆柱坐标
        theta2 = point_all(edge(idx ,2),1);
        z2 = point_all(edge(idx ,2),2);
        xi1 = theta1+2*pi;  xi2 = theta2;
        % 测地线坐标
        u = linspace(xi1,xi2,50);
        x = r_cylinder*cos(u);
        y = r_cylinder*sin(u);
        z = (z2-z1)/(xi2-xi1)*u + (z1*xi2-z2*xi1)/(xi2-xi1);
    else %测地线满足长度条件
        idx = edge_candidate(i,1); %离散点对索引
        %起点圆柱坐标
        theta1 = point_all(edge(idx ,1),1);
        z1 = point_all(edge(idx ,1),2);
        %终点圆柱坐标
        theta2 = point_all(edge(idx ,2),1);
        z2 = point_all(edge(idx ,2),2);
        % 测地线坐标
        u = linspace(theta1,theta2,50);
        x = r_cylinder*cos(u);
        y = r_cylinder*sin(u);
        z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
    end
    plot3(x,y,z,"LineWidth",1)
end
end