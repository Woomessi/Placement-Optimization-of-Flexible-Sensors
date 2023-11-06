clear
clc
close all
clear options;
addpath(genpath('C:\projects\MATLAB\robot_sensor'))

load("all_first_spot.mat")
load("all_middle_spot.mat")
load("all_last_spot.mat")
load("index_geodesic.mat")

size_all_points_seq = index_geodesic-1;
repeated_geodesic = zeros(1,size_all_points_seq);
idx_geodesic = 1:size_all_points_seq;
idx_repeated_geodesic = 1;
tol = 0.005;
for i = 1:size_all_points_seq-1
    for j = i+1:size_all_points_seq
        difference_first_spot = abs(all_first_spot(:,idx_geodesic(i)) - all_first_spot(:,idx_geodesic(j)));
        difference_middle_spot = abs(all_middle_spot(:,idx_geodesic(i)) - all_middle_spot(:,idx_geodesic(j)));
        difference_last_spot = abs(all_last_spot(:,idx_geodesic(i)) - all_last_spot(:,idx_geodesic(j)));
        if difference_first_spot(1) < tol && difference_first_spot(2) < tol && difference_first_spot(3) < tol
            if difference_middle_spot(1) < tol && difference_middle_spot(2) < tol && difference_middle_spot(3) < tol
                if difference_last_spot(1) < tol && difference_last_spot(2) < tol && difference_last_spot(3) < tol
                    idx_geodesic(i) = 0;
                    break
                end
            end
        end
    end
end