function plotRobotMultisensor(my_robot,config,size_spot,tform_spot,h_cone, fov_horizontal,target1)
%UNTITLED6 此处显示有关此函数的摘要
%   此处显示详细说明

for i = 1:size_spot
    scatter3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4),18,"magenta",'filled')
    % 绘制拟合路径点坐标系
    quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
        tform_spot{1,i}(1,1),tform_spot{1,i}(2,1),tform_spot{1,i}(3,1),0.05,'Color','r');

    quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
        tform_spot{1,i}(1,2),tform_spot{1,i}(2,2),tform_spot{1,i}(3,2),0.05,'Color','g');

    quiver3(tform_spot{1,i}(1,4),tform_spot{1,i}(2,4),tform_spot{1,i}(3,4), ...
        tform_spot{1,i}(1,3),tform_spot{1,i}(2,3),tform_spot{1,i}(3,3),0.05,'Color','b');
end
plotFoV(h_cone, fov_horizontal, tform_spot, size_spot)
% hold off
end