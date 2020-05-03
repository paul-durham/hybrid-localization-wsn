% PIscan.m
% high resolution PI scan
% © 2019 Paul Durham, School of Computer Science, Carleton University
% 
function [r0, r1] = PIscan(pta, ptl, l, s, std, mvp, ua)
% This function does PI scan using 3 sides of triangle with variable side
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% pta = actual location of node
% ptl = localized estimate for node
% l = iteration number (starting at 0)
% s = triangle side length
% std = log normal stardard deviation
% mvp = moving average point count
% ua = uav

% uav height in metres
global HT;
% wavelength of network frequency
global WLN;
% wavelength of charging frequency
global WLC;
% height of triangle
h = s*(3^.5)/2;
% original estimate error
D = [ptl; pta];
oer = pdist(D, 'euclidean');

p1 = ptl + [-s/2, -h/3];
p2 = ptl + [0, 2*h/3];
p3 = ptl + [s/2, -h/3];

% edge p1 - p2

% vector from p1 to p2
vec = p2-p1;

% distance from p1 to p
d = 0;

i = 1;
% ns is number of samples per side of triangle
ns = 21;
d12 = zeros(1,ns);
rss12 = zeros(1, ns);
for i=0:ns-1
    % current position
    pt = p1 + i * vec/(ns-1);
    % distance to actual node in metres
    D = [pt; pta];
    r = pdist(D, 'euclidean') * 100;
    d12(i+1) = r;
    % calculate rss in dbuW using 2 plane formula
    rss = 10*log(1000000 * 10.76 * r^4 / ((HT^2 + r^2))^3 * (WLN^2)/(16*pi^2));
    % add log normal fading var=4
    lnf = std*rand;
    rss12(i+1) = rss + lnf;
    
    % move uav
    UAVmove(pt, ua);
end

% take moving mean
rss12m = movmean(rss12, mvp);

% edge p2 - p3

% vector from p2 to p3
vec = p3-p2;

% distance from p2 to p
d = 0;
i = 1;
rss23 = zeros(1, ns);

for i=0:ns-1
    % current position
    pt = p2 + i * vec/(ns-1);
    % distance to actual node in metres
    D = [pt; pta];
    r = pdist(D, 'euclidean') * 100;
    % calculate rss in dbuW using 2 plane formula
    rss = 10*log(1000000 * 10.76 * r^4 / ((HT^2 + r^2))^3 * (WLN/(4*pi))^2);
    % add random fading var=4
    rss23(i+1) = rss + std*randn;
    
    % move uav
    UAVmove(pt, ua);
end

% take moving mean
rss23m = movmean(rss23, mvp);

% edge p3 - p1

% vector from p3 to p1
vec = p1-p3;

% distance from p3 to p
d = 0;
i = 1;
rss31 = zeros(1, ns);
op = p2;
for i=0:ns-1
    % current position
    pt = p3 + i * vec/(ns-1);
    % distance to actual node in metres
    D = [pt; pta];
    r = pdist(D, 'euclidean') * 100;
    % calculate rss in dbuW using 2 plane formula
    rss = 10*log(1000000 * 10.76 * r^4 / ((HT^2 + r^2))^3 * (WLN/(4*pi))^2);
    % add random fading var=4
    rss31(i+1) = rss + std*randn;
    
    % move uav
    UAVmove(pt, ua);
end

% take moving mean
rss31m = movmean(rss31, mvp);

% find max value of rss12m, rss23m
% exclude first and last elements
%rss12m(2:ns-1)
%rss23m(2:ns-1)
[max12, idx12] = max(rss12m);
[max23, idx23] = max(rss23m);
[max31, idx31] = max(rss31m);

% compute position based on sides 1-2 and 2-3
r123 = Intersect(p1, p2, p3, idx12, idx23, ns);
D = [r123;pta];
err = pdist(D, 'euclidean');

sr = 3^.5*s/6;

fprintf("pta = %f %f, ptl = %f %f, oer=%f, X123 = %f %f, s=%f, sr=%f, err = %f\n", pta, ptl, oer, r123, s, sr, err);

if (intriangle(r123, p1, p2, p3) == 0)
    fprintf("Above not in triangle\n");
end

% compute position based on sides 2-3 and 3-1
r231 = Intersect(p2, p3, p1, idx23, idx31, ns);
err = pdist2(r231, pta, 'euclidean');

fprintf("pta = %f %f, ptl = %f %f, oer=%f, X231 = %f %f, s=%f, sr=%f, err = %f\n", pta, ptl, oer, r231, s, sr, err);

if (intriangle(r231, p1, p2, p3) == 0)
    fprintf("Above not in triangle\n");
end

% compute position based on sides 3-1 and 1-2
r312 = Intersect(p3, p1, p2, idx31, idx12, ns);
err = pdist2(r312, pta, 'euclidean');

fprintf("pta = %f %f, ptl = %f %f, oer=%f, X312 = %f %f, s=%f, sr=%f, err = %f\n", pta, ptl, oer, r312, s, sr, err);

if (intriangle(r312, p1, p2, p3) == 0)
    fprintf("Above not in triangle\n");
end

% compute average point
ravg = mean([r123; r231; r312]);
err = pdist2(ravg, pta, 'euclidean');

fprintf("pta = %f %f, ptl = %f %f, oer=%f, Xxxx = %f %f, s=%f, sr=%f, err = %f\n", pta, ptl, oer, ravg, s, sr, err);

% r0 is found flag
r0 = 0;

% default return
r1 = ravg;

if (l == 0)
    % use average of estimates
    r = pdist([ravg;pta], 'euclidean') * 100;
    
    % move uav
    UAVmove(ravg, ua);
    [r0,r1] = TOClocate3(pta, ravg, ua);
    if (r0 > 0)
        % found
        r1 = ravg;
        return;
    end
else
    % check average and all 3 estimates
    % move uav
    UAVmove(ravg, ua);
    [r0,r1] = TOClocate3(pta, ravg, ua);
    if (r0 > 0)
        % found
        return;
    end
    
    % move uav
    UAVmove(r123, ua);
    [r0,r1] = TOClocate3(pta, r123, ua);
    if (r0 > 0)
        % found
        return;
    end
    
    % move uav
    UAVmove(r312, ua);
    [r0,r1] = TOClocate3(pta, r312, ua);
    if (r0 > 0)
        % found
        return;
    end
    
    % move uav
    UAVmove(r231, ua);
    [r0,r1] = TOClocate3(pta, r231, ua);
    if (r0 > 0)
        % found
        return;
    end
end
end



function t = intriangle(P, P1, P2, P3)
s = det([P1-P2;P3-P1]);
t = s*det([P3-P;P2-P3])>=0 & s*det([P1-P;P3-P1])>=0 & s*det([P2-P;P1-P2])>=0;
end


   