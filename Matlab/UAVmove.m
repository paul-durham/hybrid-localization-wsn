% UAVmove.m
% move UAV and track distance travelled
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
function UAVmove(pt, ua)
% move uav
% pt = new point
% ua = uav object

% total distance in metres
global uavdist;
global uavmvct;
global uavprev;

d = pdist([pt;uavprev], 'euclidean') * 100;
uavdist = uavdist + d;
uavmvct = uavmvct + 1;

ua.Matrix = makehgtform('translate',[pt(1) pt(2), 0]);
drawnow

uavprev = pt;

end