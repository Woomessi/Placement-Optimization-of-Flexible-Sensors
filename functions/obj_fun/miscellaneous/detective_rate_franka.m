function detective_rate = detective_rate_franka(idx_helix1, idx_helix2, idx_helix3, idx_helix4, idx_helix5, idx_helix6, idx_helix7, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, X_link, F_link, edge_candidate_all1, a, d, alpha, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone)
%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%
idx_helix = [idx_helix1, idx_helix2, idx_helix3, idx_helix4, idx_helix5, idx_helix6, idx_helix7];

detection_times = 0; % 检测到目标的次数
for idx_config = 1:size_sim

    %%%%%%%%%%%%%%%%%%%%%%
    %%% 检测目标坐标信息 %%%
    %%%%%%%%%%%%%%%%%%%%%%

    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; % 检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; % 平移变换矩阵
    target_homo1 = tform_target*target_homo;
    target1 = target_homo1(1:3,:); % 平移变换后的检测目标

    %%%%%%%%%%%%%%%%%%%%%
    %%% 当前关节角配置 %%%
    %%%%%%%%%%%%%%%%%%%%%

    q = q_all(idx_config,:); % 当前关节空间角
    link_name = ["panda_link0" "panda_link1" "panda_link2" "panda_link3" "panda_link4" "panda_link5" "panda_link6" "panda_link7"];
    config = homeConfiguration(my_robot); % 关节空间配置结构体生成
    for i = 1:size_joint % 遍历每个关节
        config(i).JointPosition = q(1,i);
    end

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% 各连杆传感器配置 %%%
    %%%%%%%%%%%%%%%%%%%%%%%

    for idx_link = 1:size_joint % 遍历所有连杆 (末端连杆对应空矩阵)
        % for idx_link = size_joint
        transform = getTransform(my_robot,config,link_name(idx_link)); %当前位形空间下连杆相对于基坐标系的齐次变换矩阵
        X_update = transform*X_link{1,idx_link}; % 当前时刻连杆的mesh坐标
        F = (double(F_link{1,idx_link}))'; % 当前连杆mesh的三角面片信息
        % size_F = size(F,2); % 三角面数量
        X = (double(X_update(1:3,:))); % 当前连杆mesh的顶点坐标信息
        % size_X = size(X,2); % 三角面片顶点数量

        TR = triangulation(F',X'); % MATLAB官方的三角面片数据结构

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 检查连杆三角面片模型 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        % figure(1);
        % trimesh(TR);
        % view(135,7);
        % axis equal
        % hold on

        % show(my_robot,config); % 打开figure属性

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 连杆mesh模型几何信息提取 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        N_f = faceNormal(TR); % 计算三角面法向量
        N_v = vertexNormal(TR); % 计算顶点法向量

        C = incenter(TR); % 计算三角面形心
        X_refined = [X,C']; % 向顶点坐标矩阵中加入三角面形心坐标
        N_refined = [N_v;N_f]; % 向顶点法向量矩阵中加入三角面法向量，作为三角面片形心法向量

        % 不引入形心
        % X_refined = X; % 向顶点坐标矩阵中加入三角面形心坐标
        % N_refined = N_v; % 向顶点法向量矩阵中加入三角面法向量，作为三角面片形心法向量

        % quiver3(C(:,1),C(:,2),C(:,3), ...
        %        N_f(:,1), N_f(:,2), N_f(:,3),5,'Color','C');
        %
        % quiver3(X(1,:),X(2,:),X(3,:), ...
        %        N_v(:,1)', N_v(:,2)', N_v(:,3)',5,'Color','M');

        %%%%%%%%%%%%%%%%%%%
        %%% Heat Method %%%
        %%%%%%%%%%%%%%%%%%%

        pstart = edge_candidate_all1{1,idx_link}(idx_helix(idx_link),1); % 起点序号
        phi = getGeodesicDistanceHeat(X,F,pstart);

        %%%%%%%%%%%%%%%%%%%%%
        %%% 计算测地线路径 %%%
        %%%%%%%%%%%%%%%%%%%%%

        pend = edge_candidate_all1{1,idx_link}(idx_helix(idx_link),2); % 终点序号
        %
        % options.method = 'continuous';
        % options.method = 'discrete';
        %
        % options.face_vertex_color = phi;
        paths = compute_geodesic_mesh(phi, X, F, pend);
        paths = unique(paths',"rows","stable");
        paths = paths';

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%
        % plot_fast_marching_mesh(X, F, phi, paths);

        % [phi,paths] = heatMethod(X,F,pstart,pend); % 用Heat Method求测地线距离与路径

        %%%%%%%%%%%%%%%%%%%%%
        %%% 路径拟合与提取 %%%
        %%%%%%%%%%%%%%%%%%%%%

        % scatter3(paths(1,:),paths(2,:),paths(3,:),18,'magenta','filled') % 绘制路径点

        size_interval = 200; % 拟合路径点间距数
        interval1 = (max(paths(1,:))-min(paths(1,:)))/size_interval; % 拟合路径点间距
        interval2 = (max(paths(2,:))-min(paths(2,:)))/size_interval; % 拟合路径点间距
        interval3 = (max(paths(3,:))-min(paths(3,:)))/size_interval; % 拟合路径点间距

        % 选择间距最大的方向作为插值方向，提高插值效果
        pp = [min(paths(1,:)):interval1:max(paths(1,:));min(paths(2,:)):interval2:max(paths(2,:));min(paths(3,:)):interval3:max(paths(3,:))];
        [~,idx_pp] = max([max(paths(1,:))-min(paths(1,:)),max(paths(2,:))-min(paths(2,:)),max(paths(3,:))-min(paths(3,:))]);

        coefficient_x = polyfit(paths(idx_pp,:),paths(1,:),5); % y-x三次多项式曲线拟合系数
        x = polyval(coefficient_x,pp(idx_pp,:)); % 拟合路径点的y坐标
        coefficient_y = polyfit(paths(idx_pp,:),paths(2,:),5); % y-x三次多项式曲线拟合系数
        y = polyval(coefficient_y,pp(idx_pp,:)); % 拟合路径点的y坐标
        coefficient_z = polyfit(paths(idx_pp,:),paths(3,:),5); % z-x三次多项式曲线拟合系数
        z = polyval(coefficient_z,pp(idx_pp,:)); % 拟合路径点的z坐标

        % x = makima(paths(idx_pp,:),paths(1,:),pp(idx_pp,:));
        % y = makima(paths(idx_pp,:),paths(2,:),pp(idx_pp,:));
        % z = makima(paths(idx_pp,:),paths(3,:),pp(idx_pp,:));

        % x = spline(paths(idx_pp,:),paths(1,:),pp(idx_pp,:));
        % y = spline(paths(idx_pp,:),paths(2,:),pp(idx_pp,:));
        % z = spline(paths(idx_pp,:),paths(3,:),pp(idx_pp,:));

        point_path = [x;y;z]; % 拟合路径点
        % point_path = paths; % 不使用拟合路径点

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 剔除在当前连杆范围之外的路径点 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        x_max = max(X(1,:));
        y_max = max(X(2,:));
        z_max = max(X(3,:));

        x_min = min(X(1,:));
        y_min = min(X(2,:));
        z_min = min(X(3,:));

        for i = 1:size(point_path,2)
            if point_path(1,i) > x_max || point_path(1,i) < x_min
                point_path(:,i) = zeros(3,1);
            else
                if point_path(2,i) > y_max || point_path(2,i) < y_min
                    point_path(:,i) = zeros(3,1);
                else
                    if point_path(3,i) > z_max || point_path(3,i) < z_min
                        point_path(:,i) = zeros(3,1);
                    end
                end
            end
        end
        point_path(:,all(point_path==0,1))=[]; % 剔除全0列
        size_point_path = size(point_path,2); % 最终的拟合路径点数量

        % scatter3(point_path(1,:),point_path(2,:),point_path(3,:),18,'magenta','filled') % 绘制路径点
        % plot3(point_path(1,:),point_path(2,:),point_path(3,:),'LineWidth',1,'Color','m') % 绘制拟合路径

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% 拟合路径点坐标系建立 %%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        % 在三角面片顶点与形心点中提取距各拟合路径点最近的点
        ID_all = dsearchn(X_refined',point_path');

        % 将各最近点的法向量作为相应路径点的法向量 (x轴基向量)
        x_point_path = N_refined(ID_all,:)';

        % 初步选定相邻路径点之间的连线为坐标系的y轴基向量
        y_point_path = zeros(3,size_point_path);
        for i = 1:size_point_path-1
            y_point_path(:,i) = point_path(:,i+1) - point_path(:,i);
            y_point_path(:,i) = y_point_path(:,i)./norm(y_point_path(:,i));
        end
        y_point_path(:,size_point_path) = y_point_path(:,size_point_path-1);

        % 通过叉乘获得z轴基向量
        z_point_path = cross(x_point_path,y_point_path);
        z_point_path = z_point_path./vecnorm(z_point_path);

        % 通过叉乘修正y轴基向量
        y_point_path = cross(z_point_path,x_point_path);

        % 绘制拟合路径点坐标系
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     x_point_path(1,:),x_point_path(2,:),x_point_path(3,:),0.5,'Color','r');
        % 
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     y_point_path(1,:),y_point_path(2,:),y_point_path(3,:),0.5,'Color','g');
        % 
        % quiver3(point_path(1,:),point_path(2,:),point_path(3,:), ...
        %     z_point_path(1,:),z_point_path(2,:),z_point_path(3,:),0.5,'Color','b');

        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        %坐标变换
        tform_link = eye(4);
        for i = 1:idx_link
            tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
        end
        % position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

        % 在拟合路径点中提取均分点，作为TOF模块点，并计算相应齐次变换矩阵
        tform_spot_all = getUniformPointFrames(size_spot, point_path, size_point_path, x_point_path, y_point_path, z_point_path);

        %%%%%%%%%%%%%%%
        %%% ToF检测 %%%
        %%%%%%%%%%%%%%%

        for idx_point_target = 1:size_point_target
            flag = 0; %循环跳出标识
            point_target = target1(:,idx_point_target); %目标点设置
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot_all{1,idx_spot};
                if tform_spot_current(:,1) == zeros(4,1)
                    break;
                end
                vt = point_target - tform_spot_current(1:3,4); %圆锥顶点到目标点的向量
                l_vt = norm(vt);
                centerline = tform_spot_current(1:3,1); %圆锥中心线
                cos_theta = dot(vt,centerline)/(l_vt*norm(centerline)); %夹角余弦
                if cos_theta > cosd(min(fov_vertical,fov_horizontal)/2) %夹角是否小于视场角的一半？
                    if l_vt*cos_theta < h_cone %测距值在中心线方向的投影距离是否在量程内
                        % range_all(idx_spot, idx_point_target) = norm(vt);
                        % range_old(idx_spot, 1) = range_now(idx_spot, 1);
                        % if l_vt < range_old(idx_spot, 1)
                        %     range_now(idx_spot, 1) = l_vt;
                        % end
                        flag = 1;
                        detection_times = detection_times + 1;
                        break
                    end
                end
            end
            if flag == 1 %跳出两重循环
                break
            end
        end

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%

        % plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot)
        % scatter3(target1(1,:),target1(2,:),target1(3,:),2,"magenta","filled")
        % view(0,90)
        % hold off
    end
end
detective_rate = -detection_times/(size_sim*size_joint);
end