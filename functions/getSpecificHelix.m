function position_helix = getSpecificHelix(edge_candidate, idx_geodesic, point_all, edge, r_cylinder, tform)
if edge_candidate(idx_geodesic,1)==0 %共轭测地线
    idx = edge_candidate(idx_geodesic,2);
    
    theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
    z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
    theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
    z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标
    %生成圆柱螺旋线笛卡尔坐标
    xi1 = theta1+2*pi;  xi2 = theta2;
    u = linspace(xi1,xi2,500);
    x = r_cylinder*cos(u);
    y = r_cylinder*sin(u);
    z = (z2-z1)/(xi2-xi1)*u + (z1*xi2-z2*xi1)/(xi2-xi1);
else
    idx = edge_candidate(idx_geodesic,1); %测地线

    theta1 = point_all(edge(idx,1),1); %起点角度圆柱坐标
    z1 = point_all(edge(idx,1),2); %起点高度圆柱坐标
    theta2 = point_all(edge(idx,2),1); %终点角度圆柱坐标
    z2 = point_all(edge(idx,2),2); %终点高度圆柱坐标

    % 生成圆柱螺旋线笛卡尔坐标
    u = linspace(theta1,theta2,500); %均分角度
    x = r_cylinder*cos(u);
    y = r_cylinder*sin(u);
    z = (z2-z1)/(theta2-theta1)*u + (z1*theta2-z2*theta1)/(theta2-theta1);
end

position_helix_local = [x;y;z;ones(1,500)]; %当地坐标系下的螺旋线笛卡尔坐标
position_helix = tform*position_helix_local; %世界坐标系下的螺旋线笛卡尔坐标
end