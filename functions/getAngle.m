function angle = getAngle(vector1,vector2)
%GETANGLE 此处显示有关此函数的摘要
%   此处显示详细说明
angle = acos(dot(vector1,vector2)/(norm(vector1)*norm(vector2)));
end

