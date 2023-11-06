function T = getTformMDH(a,d,alpha,q)
%TFORM_DH 基于DH参数的齐次变换矩阵
%   此处显示详细说明
T = [cos(q), -sin(q), 0, a;
     sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d;
     sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d;
     0, 0, 0, 1];
end