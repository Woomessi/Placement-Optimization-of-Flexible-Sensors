function plotHelixCell(r_cylinder, h_cylinder, size_alternative_edge, alternative_edge, point_all, edge)
%作圆柱图
[X,Y,Z] = cylinder(r_cylinder,500); 
surf(X,Y,-Z);
set(findobj('Type','surface'),'FaceColor',[0.85,0.85,0.85],'MeshStyle','row')
axis equal
zlim([-h_cylinder 0]);
grid off; 
hold on;

%作测地线图
for i = 1:size_alternative_edge
    if alternative_edge(i,1) == 0 %共轭测地线满足长度条件
        idx = alternative_edge(i,2); %离散点对索引
        %起点圆柱坐标
        theta1 = point_all{edge(idx ,1)}(1); 
        z1 = point_all{edge(idx ,1)}(2);
        %终点圆柱坐标
        theta2 = point_all{edge(idx ,2)}(1);
        z2 = point_all{edge(idx ,2)}(2);
        
        % %起点圆柱坐标
        % theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
        % z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
        % %终点圆柱坐标
        % theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
        % z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标
        
        xi1 = theta1+2*pi;  xi2 = theta2;
        
        % 测地线坐标
        u = linspace(xi1,xi2,500);
        x = r_cylinder*cos(u);
        y = r_cylinder*sin(u);
        z = (z2-z1)/(xi2-xi1)*u + (z1*xi2-z2*xi1)/(xi2-xi1);
    else %测地线满足长度条件
        idx = alternative_edge(i,1); %离散点对索引
        %起点圆柱坐标
        theta1 = point_all{edge(idx ,1)}(1);
        z1 = point_all{edge(idx ,1)}(2);
        %终点圆柱坐标
        theta2 = point_all{edge(idx ,2)}(1);
        z2 = point_all{edge(idx ,2)}(2);

        % %起点圆柱坐标
        % theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
        % z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
        % %终点圆柱坐标
        % theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
        % z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标

        % 测地线坐标
        u = linspace(theta1,theta2,500);
        x = r_cylinder*cos(u);
        y = r_cylinder*sin(u);
        z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
    end
    plot3(x,y,z);
end
end