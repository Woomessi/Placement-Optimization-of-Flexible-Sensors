function detective_rate = detective_rate_simple(x,target,joint_constraint,a,d,alpha,edge_candidate,point_all,edge, r_cylinder,size_point_target,fov_vertical,fov_horizontal,size_sim,h_cone,idx_link)
% clear
% clc
% close all
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%
%%% 蒙特卡洛仿真 %%%
%%%%%%%%%%%%%%%%%%%
% range_each_sim = zeros(x(1), size_sim);
%利用随机数生成关节空间配置
q_all = joint_constraint(1,:)*(pi/180) + (joint_constraint(2,:) - joint_constraint(1,:)).*(pi/180).*rand(size_sim,1);

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

    position_helix = getSpecificHelix(edge_candidate, x(2), point_all, edge, r_cylinder, tform_link); %生成特定螺旋线
    [~, tform_spot_all] = getSpotFrame(x(1), position_helix, tform_link, position_link, r_cylinder); %生成传感器点相对于世界坐标系的齐次变换矩阵

    %%%%%%%%%%%%%%%%%%%
    %%% ToF模块测距 %%%
    %%%%%%%%%%%%%%%%%%%

    % detection_times = getRange(size_point_target, target, x(1), tform_spot_all, fov_vertical, fov_horizontal, h_cone, detection_times);
    % range_all = ones(x(1),size_point_target); %大型稀疏矩阵，数据结构待优化
    % range_now = ones(x(1),1);
    % range_old = ones(x(1),1);
    
    for idx_point_target = 1:size_point_target
        flag = 0; %循环跳出标识
        point_target = target(:,idx_point_target); %目标点设置
        for idx_spot = 1:x(1)
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
  
    % range = min(range_all,[],2);
    % range_each_sim(:,idx_config) = min(range_all,[],2);
end
detective_rate = -detection_times/size_sim; %在优化中求最大值
%%%%%%%%%%%%
%%% 作图 %%%
%%%%%%%%%%%%

% config = homeConfiguration(my_robot); %关节空间配置结构体生成
% for i = 1:size_joint %遍历每个关节
%     config(i).JointPosition = q(idx_config,i);
% end
% show(my_robot,config)
% plotFoV(h_cone, fov_horizontal, tform_spot_all, x(1));
% scatter3(target(1,:),target(2,:),target(3,:),2,"magenta","filled")
% hold off
end