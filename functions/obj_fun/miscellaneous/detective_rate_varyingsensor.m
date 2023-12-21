function detective_rate = detective_rate_varyingsensor(idx_helix1, idx_helix2, idx_helix3, idx_helix4, idx_helix5, idx_helix6, idx_helix7, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, size_joint, a, d, alpha, edge_candidate, point_all, edge, r_cylinder, size_spot, size_point_target, fov_vertical, fov_horizontal, h_cone, my_robot)

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

idx_helix = [idx_helix1, idx_helix2, idx_helix3, idx_helix4, idx_helix5, idx_helix6, idx_helix7];
detection_times = 0; %检测到目标的次数
parfor idx_config = 1:size_sim %当前关节角配置

    %%%%%%%%%%%%%%%%
    %%% 检测目标 %%%
    %%%%%%%%%%%%%%%%

    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; %检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
    target_homo1 = tform_target*target_homo;
    target1 = target_homo1(1:3,:); %平移变换后的检测目标

    q = q_all(idx_config,:);

    for idx_link = 1:size_joint

        %%%%%%%%%%%%
        %%% 作图 %%%
        %%%%%%%%%%%%

        % plotHelix1(h_cylinder(idx_link), size_edge_candidate, edge_candidate, point_all, edge, r_cylinder(idx_link));

        %%%%%%%%%%%%%%%%%%%%%
        %%% 连杆坐标系定义 %%%
        %%%%%%%%%%%%%%%%%%%%%

        %坐标变换
        tform_link = eye(4);
        for i = 1:idx_link
            tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
        end
        position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

        %%%%%%%%%%%%%%%%%%%%%
        %%% 圆柱螺旋线配置 %%%
        %%%%%%%%%%%%%%%%%%%%%

        position_helix = getSpecificHelix(edge_candidate{1,idx_link}, idx_helix(idx_link), point_all{1,idx_link}, edge{1,idx_link}, r_cylinder(idx_link), tform_link); %生成特定螺旋线
        [~, tform_spot_all] = getSpotFrame(size_spot, position_helix, tform_link, position_link, r_cylinder(idx_link)); %生成传感器点相对于世界坐标系的齐次变换矩阵

        for idx_point_target = 1:size_point_target
            flag = 0; %循环跳出标识
            point_target = target1(:,idx_point_target); %目标点设置
            for idx_spot = 1:size_spot
                tform_spot_current = tform_spot_all{1,idx_spot};
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
       
        % config = homeConfiguration(my_robot); %关节空间配置结构体生成
        % for i = 1:size_joint %遍历每个关节
        %     config(i).JointPosition = q(1,i);
        % end
        % show(my_robot,config)
        % plotFoV(h_cone, fov_horizontal, tform_spot_all, size_spot);
        % scatter3(target1(1,:),target1(2,:),target1(3,:),2,"magenta","filled")
        % view(0,90)
        % hold off

    end
end
detective_rate = -detection_times/(size_sim*size_joint);
end