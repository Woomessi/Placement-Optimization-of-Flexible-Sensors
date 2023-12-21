function spot = getUniformPoints(size_sensor, position_helix, size_point_helix)
length = 0;
for i_a = 1:size_point_helix-1
    vector = position_helix(i_a+1,1:3) - position_helix(i_a,1:3);
    length = length + norm(vector);
end

%%% 初始化 %%%
length1 = 0;
interval1 = length/(size_sensor-1);
spot = zeros(size_sensor,3);% 初始化均分点

%%% 生成第一个均分点 %%%
spot(1,:) = position_helix(1,:);

%%% 生成最后一个均分点 %%%
spot(size_sensor,:) = position_helix(size_point_helix,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% 生成其余的均分点 %%%
j = 2;
for i_b = 1:size_point_helix-1
    p2 = position_helix(i_b+1,:);
    p1 = position_helix(i_b,:);
    vector1 = p2(1:3) - p1(1:3);
    length1 = length1 + norm(vector1);
    if length1 > interval1
        [~,index] = min([interval1-length1+norm(vector1),length1-interval1]);
        if index == 1
            spot(j,:) = p1;
        else
            spot(j,:) = p2;
        end
        length1 = 0;
        j = j+1;
    end
end

end
