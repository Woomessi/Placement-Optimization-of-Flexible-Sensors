function ax = setPlotStyle(scenario)
ax = show3D(scenario);
view(0,0)%正视
% view(-90,0)%左视
% view(90,90)%俯视

% view(-40,15)
zlim([0 inf])

% ax.View = [-37.5,30];
% ax.CameraPositionMode = "manual";
% ax.CameraPosition = [-6.833607033563605,-8.948799195144693,7.388762690675049];
% ax.CameraTargetMode = "manual";
% ax.CameraTarget = [0.176872113602082,0.187435105215806,0.740013558596934];
% ax.CameraViewAngleMode = "manual";
% ax.CameraViewAngle = 1.347371145846553;
% ax.XLim = [-1.933024686348753,2.631392491565355];
% ax.YLim = [-1.414744027057242,2.185255972942756];
% ax.ZLim = [-1.246737247414615,2.353262752585382];

light("Position",[-1 -1 0])
grid on
hold on
end