function plotFoV(range_max, fov_horizontal, tform_spot_all, size_spot)
[X,Y,Z]=cylinder([0 range_max*tan(deg2rad(fov_horizontal/2))], 50);
h = range_max;
Z = Z*h;
eul = [0 pi/2 0];
M1 = eul2tform(eul);

xlabel("x")
ylabel("y")
zlabel("z")
axis equal
hold on

for i = 1:size_spot
    tform_spot_current = tform_spot_all{1,i};
    if tform_spot_current(:,1) == zeros(4,1)
        break;
    end
    surf(X,Y,Z, Parent=hgtransform(Matrix=tform_spot_current*M1), LineStyle='none', FaceAlpha=0.4)
end
end