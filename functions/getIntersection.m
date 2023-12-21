function [K_Pnt] = getIntersection(P1_Pnt,L1_Dir,P2_Pnt,L2_Dir)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
% 后缀_Dir单位向量，后缀_Vec一般向量，后缀_Pnt点坐标，_Norm线段长度
%单位化
L1_Dir = L1_Dir./norm(L1_Dir);
% 直线L2方向向量，改变L2_Dir即可
%单位化
L2_Dir = L2_Dir./norm(L2_Dir);
% P1P2向量
P1P2_Vec = P2_Pnt-P1_Pnt;
%计算点P2到直线L1的距离
P2P3_Norm = norm(cross(P1P2_Vec,L1_Dir));
%投影点P3坐标
P3_Pnt = P1_Pnt + (P1P2_Vec*L1_Dir').*L1_Dir;

CosTheta= abs(L1_Dir*L2_Dir');
K_Pnt = [];%交点
if CosTheta < 1e-7
    % 两直线垂直
    % 交点坐标
    K_Pnt = P3_Pnt;
end

if CosTheta > 1e-7
    TanTheta = (1-CosTheta^2)^0.5/CosTheta;
    KP3_Norm = P2P3_Norm/TanTheta;
    
    K1_Pnt = P3_Pnt + KP3_Norm.*L1_Dir;
    K2_Pnt = P3_Pnt - KP3_Norm.*L1_Dir;
    
    P2K1_Vec = K1_Pnt - P2_Pnt;
    P2K2_Vec = K2_Pnt - P2_Pnt;
    
    D1 = norm(cross(P2K1_Vec,L2_Dir)); %点K1到直线L2的距离
    D2 = norm(cross(P2K2_Vec,L2_Dir)); %点K2到直线L2的距离
    % 到直线L2距离小的点坐标为交点
    if D1 < D2
        K_Pnt = K1_Pnt;
    else
        K_Pnt = K2_Pnt;
    end
end
end