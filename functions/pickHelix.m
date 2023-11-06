function [edge_candidate, size_edge_candidate]  = pickHelix(tol, size_edge, l_edge, l_sensor)
edge_candidate = zeros(size_edge,2); %储存满足长度条件的待选离散点对的数组
for i = 1:size_edge
    if abs(l_edge(i,1) - l_sensor) < tol %测地线长度是否接近柔性传感器环长度
        edge_candidate(i,1) = i; %保存离散点对索引
    else
        if abs(l_edge(i,2)-l_sensor) < tol %共轭测地线长度是否接近柔性传感器环长度
            edge_candidate(i,2) = i; %保存离散点对索引
        end
    end
end
edge_candidate(all(edge_candidate==0,2),:)=[]; %清除数组全0行
size_edge_candidate = size(edge_candidate,1); %待选离散点对总数
end