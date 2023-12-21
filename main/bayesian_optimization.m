clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Preprocessing for the optimization (similar to "detection_simulation.m") %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

size_sim = 3000;
size_theta_object = 25; 

fov_horizontal = 25; 
fov_vertical = 25; 
range_max = 0.5; 
h_cone = range_max*cosd(fov_horizontal/2); 

my_robot = importrobot("frankaEmikaPanda.urdf");

size_joint = 7;

joint_constraint = zeros(2,size_joint);
joint_constraint(:,1) = [-2.8973;2.8973];
joint_constraint(:,2) = [-1.7628;1.7628];
joint_constraint(:,3) = [-2.8973;2.8973];
joint_constraint(:,4) = [-3.0718;-0.0698];
joint_constraint(:,5) = [-2.8973;2.8973];
joint_constraint(:,6) = [-0.0175;3.7525];
joint_constraint(:,7) = [-2.8973;2.8973];

[size_point_target, target] = createObject(0.311/2, 1.75, size_theta_object);
target_homo = [target;ones(1,size_point_target)];
for i = 1:size_point_target
    target_homo(:,i) = [1,0,0,0;0,1,0,0;0,0,1,-0.8;0,0,0,1]*target_homo(:,i);
end
r_min = 0;
r_max = 0.855;
r_obj_offset = r_min+0.149+(r_max-r_min)*rand(size_sim,1);

ang_min = -90;
ang_max = 45;
theta_obj_offset = ang_min+(ang_max-ang_min)*rand(size_sim,1);

load("interpState2.mat",'interpStates')
q_all = [interpStates;flip(interpStates(1:39,:))];

load("all_tform_spot_link0.mat",'all_tform_spot')
all_tform_spot_link0 = all_tform_spot;
size_all_tform_spot_link0 = size(all_tform_spot_link0,2);

load("all_tform_spot_link1.mat",'all_tform_spot');
all_tform_spot_link1 = all_tform_spot;
size_all_tform_spot_link1 = size(all_tform_spot_link1,2);

load("all_tform_spot_link2.mat",'all_tform_spot');
all_tform_spot_link2 = all_tform_spot;
size_all_tform_spot_link2 = size(all_tform_spot_link2,2);

load("all_tform_spot_link3.mat",'all_tform_spot');
all_tform_spot_link3 = all_tform_spot;
size_all_tform_spot_link3 = size(all_tform_spot_link3,2);

load("all_tform_spot_link4.mat",'all_tform_spot');
all_tform_spot_link4 = all_tform_spot;
size_all_tform_spot_link4 = size(all_tform_spot_link4,2);

load("all_tform_spot_link5.mat",'all_tform_spot');
all_tform_spot_link5 = all_tform_spot;
size_all_tform_spot_link5 = size(all_tform_spot_link5,2);

load("all_tform_spot_link6_3.mat",'all_tform_spot');
all_tform_spot_link6 = all_tform_spot;
size_all_tform_spot_link6 = size(all_tform_spot_link6,2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Define the optimization variables %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

idx_geodesic0 = optimizableVariable('idx_geodesic0',[1,size_all_tform_spot_link0],'Type','integer');
idx_geodesic1 = optimizableVariable('idx_geodesic1',[1,size_all_tform_spot_link1],'Type','integer');
idx_geodesic2 = optimizableVariable('idx_geodesic2',[1,size_all_tform_spot_link2],'Type','integer');
idx_geodesic3 = optimizableVariable('idx_geodesic3',[1,size_all_tform_spot_link3],'Type','integer');
idx_geodesic4 = optimizableVariable('idx_geodesic4',[1,size_all_tform_spot_link4],'Type','integer');
idx_geodesic5 = optimizableVariable('idx_geodesic5',[1,size_all_tform_spot_link5],'Type','integer');
idx_geodesic6 = optimizableVariable('idx_geodesic6',[1,size_all_tform_spot_link6],'Type','integer');

idx_link1 = optimizableVariable('idx_link1',[0,6],'Type','integer');
idx_link2 = optimizableVariable('idx_link2',[0,5],'Type','integer');
idx_link3 = optimizableVariable('idx_link3',[1,5],'Type','integer');
idx_link4 = optimizableVariable('idx_link4',[1,4],'Type','integer');
idx_link5 = optimizableVariable('idx_link5',[2,4],'Type','integer');
idx_link6 = optimizableVariable('idx_link6',[3,4],'Type','integer');
idx_link7 = optimizableVariable('idx_link7',[0,6],'Type','integer');

% Define the number of ToF modules
size_spot = 16;

%%%%%%%%%%%%%%%%%%%%
%%% Use 1 sensor %%%
%%%%%%%%%%%%%%%%%%%%

fun = @(x)getDetectiveRate_1(x.idx_link1, x.idx_geodesic0, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, x.idx_geodesic6,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
results = bayesopt(fun,[idx_link1, idx_geodesic0, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5, idx_geodesic6],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 2 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_double(6, x.idx_link2, x.idx_geodesic0, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link2, idx_geodesic0, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 3 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_3(6, 0, x.idx_link3, 6929, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, x.idx_geodesic5, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link3, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4, idx_geodesic5],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 4 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_4(6, 0, 5, x.idx_link4, 6929, x.idx_geodesic1, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link4, idx_geodesic1, idx_geodesic2, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 5 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_5(6, 0, 5, 1, x.idx_link5, 6929, 1142, x.idx_geodesic2, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link5, idx_geodesic2, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 6 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_6(6, 0, 5, 1, 2, x.idx_link6, 6929, 1142, 20761, x.idx_geodesic3, x.idx_geodesic4, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,[idx_link6, idx_geodesic3, idx_geodesic4],'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});

%%%%%%%%%%%%%%%%%%%%%
%%% Use 7 sensors %%%
%%%%%%%%%%%%%%%%%%%%%

% fun = @(x)getDetectiveRate_7(6, 0, 5, 1, 2, 4, 3, 6929, 1142, 20761, x.idx_geodesic3, 10451, 13100, 17,  size_spot, size_point_target, size_sim, r_obj_offset, theta_obj_offset, target_homo, q_all, my_robot, size_joint, all_tform_spot_link0, all_tform_spot_link1, all_tform_spot_link2, all_tform_spot_link3, all_tform_spot_link4, all_tform_spot_link5, all_tform_spot_link6, fov_vertical, fov_horizontal, h_cone);
% results = bayesopt(fun,idx_geodesic3,'MaxObjectiveEvaluations',60,'UseParallel',true,'PlotFcn',{});