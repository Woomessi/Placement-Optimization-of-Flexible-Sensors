function tform_sensor_all = getUniformPointFrames(size_sensor, position_helix, size_point_helix, x_point_path, y_point_path, z_point_path)
length = 0;
for i_a = 1:size_point_helix-1
    vector = position_helix(:,i_a+1) - position_helix(:,i_a);
    length = length + norm(vector);
end

%%% 初始化 %%%
length1 = 0;
interval1 = length/(size_sensor-1);
spot = zeros(3,size_sensor);% 初始化均分点
x_basis = zeros(3,size_sensor);
y_basis = zeros(3,size_sensor);
z_basis = zeros(3,size_sensor);
%%% 生成第一个均分点 %%%
spot(:,1) = position_helix(:,1);
x_basis(:,1) = x_point_path(:,1);
y_basis(:,1) = y_point_path(:,1);
z_basis(:,1) = z_point_path(:,1);
%%% 生成最后一个均分点 %%%
spot(:,size_sensor) = position_helix(:,size_point_helix);
x_basis(:,size_sensor) = x_point_path(:,size_point_helix);
y_basis(:,size_sensor) = y_point_path(:,size_point_helix);
z_basis(:,size_sensor) = z_point_path(:,size_point_helix);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% 生成其余的均分点 %%%
j = 2;
for i_b = 1:size_point_helix-1
    p2 = position_helix(:,i_b+1);
    p1 = position_helix(:,i_b);
    vector1 = p2 - p1;
    length1 = length1 + norm(vector1);
    if length1 > interval1
        [~,index] = min([interval1-length1+norm(vector1),length1-interval1]);
        if index == 1
            spot(:,j) = p1;
            x_basis(:,j) = x_point_path(:,i_b);
            y_basis(:,j) = y_point_path(:,i_b);
            z_basis(:,j) = z_point_path(:,i_b);
        else
            spot(:,j) = p2;
            x_basis(:,j) = x_point_path(:,i_b+1);
            y_basis(:,j) = y_point_path(:,i_b+1);
            z_basis(:,j) = z_point_path(:,i_b+1);
        end
        length1 = 0;
        j = j+1;
    end
end
tform_sensor_all = cell(1,size_sensor);
for i = 1:size_sensor
    tform_sensor_all(1,i) = {[x_basis(:,i), y_basis(:,i) ,z_basis(:,i), spot(:,i); 0 0 0 1]};
end
end
