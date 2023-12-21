function detection_times = getRange(size_point_target, target, size_spot, tform_spot_all, fov_vertical, fov_horizontal, h_cone, detection_times)
    % range_all = ones(size_spot,size_point_target); %大型稀疏矩阵，数据结构待优化
    % range_now = ones(size_spot,1);
    % range_old = ones(size_spot,1);
    for idx_point_target = 1:size_point_target
        flag = 0; %循环跳出标识
        point_target = target(:,idx_point_target); %目标点设置
        for idx_spot = 1:size_spot
            tform_spot_current = tform_spot_all{1,idx_spot};
            vt = point_target - tform_spot_current(1:3,4); %圆锥顶点到目标点的向量
            l_vt = norm(vt);
            centerline = tform_spot_current(1:3,1); %圆锥中心线
            cos_theta = dot(vt,centerline)/(l_vt*norm(centerline)); %夹角余弦
            % cos_theta = dot(vt,tform_spot_current(1:3,1))/(l_vt*norm(tform_spot_current(1:3,1))); %夹角余弦
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