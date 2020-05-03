
% GetIntersect.m
% get augmented traces and find intersection
% © 2019 Paul Durham, School of Computer Science, Carleton University
% 
function [r0, r1, r2] = GetIntersect(o, j, k, mvp)

r0 = 0;
r1 = [0.0, 0.0];
%
rss12 = o.getRSSItrace(j);
xpos12 = o.getUxtrace(j);
ypos12 = o.getUytrace(j);

nsab = size(rss12, 1);

% take moving mean
rss12m = movmean(rss12, mvp);

[max12, idx12] = max(rss12m);
%fprintf("up max=%f at %d : %f %f\n", max12, idx12, xpos12(idx12), ypos12(idx12));

rss34 = o.getRSSItrace(k);
xpos34 = o.getUxtrace(k);
ypos34 = o.getUytrace(k);

nscd = size(rss34, 1);

if (nscd < 20)
    fprintf("wtf...\n");
end

% take moving mean
rss34m = movmean(rss34, mvp);

% find max value of rss34m
% exclude first and last elements
[max34, idx34] = max(rss34m);
%fprintf("dn max=%f at %d : %f %f\n", max34, idx34, xpos34(idx34), ypos34(idx34));

% compute position based on sides 1-2 and 3-4
pa = [xpos12(1);ypos12(1)];
pb = [xpos12(nsab);ypos12(nsab)];
pc = [xpos34(1);ypos34(1)];
pd = [xpos34(nscd);ypos34(nscd)];

% check the angle
r2 = CheckAngle(pa, pb, pc, pd);
fprintf("cos angle %d,%d = %f\n", j, k, r2);

if (abs(r2) > 0.99)
    % lines are parallel or nearly so
    return;
end

r0 = 1;
r1 = IntersectP(pa, pb, pc, pd, idx12, idx34, nsab, nscd);

end

function [r0] = CheckAngle(pa, pb, pc, pd)

% compute magnitude of ab, cd
D = [pa';pb'];
ab = pdist(D);
D = [pc';pd'];
cd = pdist(D);

% compute dot product
vab = pb - pa;
vcd = pd - pc;

D = [vab';vcd'];
cs = pdist(D, 'cosine');

dp = vab(1) * vcd (1) + vab(2) * vcd(2);

% if in the same direction dp will be equal to product of magnitudes
% r0 is cosine of angle
r0 = dp / (ab * cd);

end



   