function detective_rate = detective_rate_helix(idx_helix,size_spot,r_obj_offset,theta_obj_offset,target_homo,joint_constraint,a,d,alpha,edge_candidate,point_all,edge, r_cylinder,size_point_target,fov_vertical,fov_horizontal,size_sim,h_cone,idx_link)
% clear
% clc
% close all
% addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%

%利用随机数生成关节空间配置
q_all = zeros(size_sim,7);
for i = 1:size_sim
    q_all(i,:) = joint_constraint(1,:) + (joint_constraint(2,:) - joint_constraint(1,:)).*rand(1,size_joint);
end

detection_times = 0; %检测到目标的次数
parfor idx_config = 1:size_sim %当前关节角配置

    %%%%%%%%%%%%%%%%%%%%%
    %%% 连杆坐标系定义 %%%
    %%%%%%%%%%%%%%%%%%%%%

    q = q_all(idx_config,:);
    %坐标变换
    tform_link = eye(4);
    for i = 1:idx_link
        tform_link = tform_link*getTformMDH(a(i),d(i),alpha(i),q(i));
    end
    position_link = tform_link(1:3,4)';%Joint_Link坐标系原点在基坐标系中的坐标

    %%%%%%%%%%%%%%%%%%%%%
    %%% 圆柱螺旋线配置 %%%
    %%%%%%%%%%%%%%%%%%%%%

    position_helix = getSpecificHelix(edge_candidate, idx_helix, point_all, edge, r_cylinder, tform_link); %生成特定螺旋线
    [~, tform_spot_all] = getSpotFrame(size_spot, position_helix, tform_link, position_link, r_cylinder); %生成传感器点相对于世界坐标系的齐次变换矩阵

    %%%%%%%%%%%%%%%%
    %%% 检测目标 %%%
    %%%%%%%%%%%%%%%%

    translation = [r_obj_offset(idx_config)*cosd(theta_obj_offset(idx_config));r_obj_offset(idx_config)*sind(theta_obj_offset(idx_config));0]; %检测目标平移向量
    tform_target = [eye(3),translation;0 0 0 1]; %平移变换矩阵
    target_homo1 = tform_target*target_homo;
    target1 = target_homo1(1:3,:); %平移变换后的检测目标
    
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
end
detective_rate = -detection_times/size_sim; %在优化中求最大值
end