% LCscan.m
% move to localized coordinate for extended perpendicular intersection
% edge to edge equilateral or isosceles tilin
function [r0, r1] = LCscan(i, ml, mr, mb, mt, p1, theta, delta, hint, n, PP, u, ht, std, sf, sr, mvp, ua, maxest)
% This function scans to localized coordinate of node
% if lc is not within charging footprint it will scan u/2 more
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% r0 = 1: in charging area
% r0 = 0: not in charging area
% r1: localized coordinates
%
% i = node index
% p1 = starting point
% delta = distance between moves
% hint = hello interval, number of deltas between hellos
% theta = starting angle in units of pi/3
% n = node count
% PP = array of nodes
% u = unit disk radius
% ht = UAV hight in metres
% std = log normal standard deviation
% sf = scale factor metres/unit distance
% sr = sample rate samples/metre
% mvp = moving average point count
% ua = uav
% maxest = maxumim number of estimates to try before refined PI

% most recent localized coordinates
global LC;
% error of LC against actual location
global ER;
% up trace used for latest localization
global UP;
% down trace used for latest localization
global DN;
% trace count used for latest localization
global TC;
% locaization count
global LT;
% scan upangle
global UPANGLE;
% scan downangle
global DNANGLE;

global LC1;

% charging footprint radius
global CHGRAD;

global WLN;
global WLC;
% wavelength of network frequency


% starting point
pt = p1;

% get actual coordinates
pta(1) = PP(1,i);
pta(2) = PP(2,i);

% get WSNnode
o = matlabJava.WSNnode.getnode(i);

% get the longest up and down traces
nu = o.getUpDown(1);
nd = o.getUpDown(-1);

% compute new localized coordinate based on all available traces
[~, r123, ~] = GetIntersect(o, nu, nd, mvp);

LC(1,i) = r123(1);
LC(2,i) = r123(2);

% localized coordinates
ptl(1) = LC(1,i);
ptl(2) = LC(2,i);

% distance to localized coordinate
D = [pt; ptl];
dl = pdist(D);

% angle
angle = atan2(ptl(2)-p1(2), ptl(1)-p1(1));
aba = abs(angle);

if ((aba > pi/3 * 0.8 && aba < pi/3 * 1.2) || (aba > pi/1.5 * 0.8 && aba < pi/1.5 * 1.2))
    angleOK = true;
else
    angleOK = false;
end

fprintf("angle=%f, degrees=%f, ok=%d\n", angle, angle / pi * 180.0, angleOK);

% calculate number of moves to localized coordinate
nsl = uint16(dl / delta);

% default value of r0
r0 = 0;

% move to localized coordinate
for j=1:nsl
    % move UAV another increment
    xinc = delta*cos(angle);
    yinc = delta*sin(angle);
    pt = pt + [xinc, yinc];
    if (mod(j,hint) == 1)
        UAVmove(pt, ua);
    end
    
    % send hello
    % don't execute this for now
    %if (0 && mod(j,hint) == 1)
    if (angleOK && mod(j,hint) == 1)
        for k=1:n
            % exclude target node
            if (k ~= i)
                % get WSNnode
                p = matlabJava.WSNnode.getnode(k);
                % get node coordinates
                ptn(1) = PP(1,k);
                ptn(2) = PP(2,k);
                D = [pt; ptn];
                r = pdist(D);
                if (r<=u)
                    % node is within radius
                    
                    % distance to actual node in metres
                    r = r * 100;
                    % calculate rss in dbuW using 2 plane formula
                    rss = 10*log(1000000 * 10.76 * r^4 / ((ht^2 + r^2))^3 * (WLN^2)/(16*pi^2));
                    
                    % add log normal fading
                    lnf = std * randn * r/(r + ht);
                    rss = rss + lnf;
                    
                    rc = p.addRSSI(pt(1), pt(2), rss);
                end
                if (j + hint >= nsl+1)
                    % close RSSI traces at last point
                    rc = p.endRSSI(false);
                    if (rc < 0)
                        %fprintf("node %d: %f %f, uav %f %f, rc = %d: trace invalid %f, %f\n", k, pta(1), pta(2), pt(1), pt(2), rc, r, u);
                    else
                        if (rc > 0)
                            %fprintf("node %d: %f %f, uav %f %f, rc = %d: trace OK %f, %f\n", k, pta(1), pta(2), pt(1), pt(2), rc, r, u);
                        else
                            if (rc == 0)
                                %fprintf("node %d: %f %f, uav %f %f, rc = %d: trace n.i.p. %f, %f\n", k, pta(1), pta(2), pt(1), pt(2), rc, r, u);
                            end
                        end
                    end
                end
            end
        end
    end
end

% is UAV over charging footprint?
% if true, return new localized coordinate
[r0, r1] = TOClocate3(pta, pt, ua);
if (r0 == 1)
    %    LC(1,i) = r1(1);
    %    LC(2,i) = r1(2);
    fprintf("%d check ToC orig %f.%f\n", i, r1);
    LC1 = LC1+1;
    %    return;
end

% get numer of traces
tup = o.getAll(1);
tdn = o.getAll(-1);

nt = o.getTraceCnt();

%if(nt == TC(i))
% no new traces - try alternate trace if available
%    if (nt == 2)
%        return;
%    end
%end

o.dumpNode();

% new traces
% recompute location based on current best 2 traces
%nu = o.getBestUp();
%nd = o.getBestDown();
%pt = GetIntersect(o, nu, nd, mvp);
%UAVmove(pt, ua);
%fprintf("%d recompute %d,%d: %f,%f\n", i, nu, nd, pt);
%[r0, r1] = TOClocate(pta, pt, ua);
%if (r0 == 1)
%    LC(1,i) = r1(1);
%    LC(2,i) = r1(2);
%    UP(i) = nu;
%    DN(i) = nd;
%    TC(i) = nt;
%    return;
%end

pav = [0.0,0.0];
nav = 0;
%nloc = nt*(nt+1)/2;

%locar = zeros(nloc, 7);
locar = [];
if (i == 8888)
    figure();
end

% look at all combination of up and down
for j=1:nt
    for k=j+1:nt
        %if (j ~= UP(i))
        [r0, rx, ca] = GetIntersect(o, j, k, mvp);
        if (r0 == 1)
            err = pdist2(rx, pta, 'euclidean');
            dst = pdist2(rx, ptl, 'euclidean');
            fprintf("alt %d,%d: err=%f, lc=%f,%f\n", j, k, err, rx);
            pav = pav + rx;
            nav = nav + 1;
            locvc = zeros(1,7);
            locvc(1) = abs(ca); % cos of angle
            locvc(2) = err; % distance from actual loc
            locvc(3) = dst; % distance from first estimate
            locvc(4) = j;
            locvc(5) = k;
            locvc(6) = rx(1); % current estimate
            locvc(7) = rx(2);
            if (i == 8888)
                viscircles(rx,CHGRAD);
                axis equal;
                if (rx == ptl)
                    hold on;
                    scatter(rx(1), rx(2), 'r');
                else
                    hold on;
                    scatter(rx(1), rx(2), 'b', '*');
                end
                scatter(pta(1), pta(2), 500, 'X', 'g');
            end
            locar = [locar ; locvc];
        end
    end
end

hold on;

if (i == 88)
    fprintf("this one\n");
end

% sort estimates by distance from first estimate
lsort = sortrows(locar, 3);

% maximum number of locations to check
maxloc = min(maxest, nav);
n = maxloc;

osort = lsort;

% perform circle reductionfor i=n:-1:1
for k=maxloc:-1:1
    rc = circlereduce(lsort, CHGRAD, k, 6, 7);
    if (rc == 1)
        fprintf("circle %d covered\n", k);
        lsort(k,:) = [];
        n = n - 1;
    end
end

if (n + 100  < maxloc)
    pt1(1) = osort(1,6);
    pt1(2) = osort(1,7);
    plotcircs(osort, maxloc, pt1, pta);
    pt1(1) = lsort(1,6);
    pt1(2) = lsort(1,7);
    plotcircs(lsort, n, pt1, pta);
end

% want to record least error among estimates tested
% initialize err to the error of the first estimate
err = lsort(1,2);
ER(i) = err;

% try estimates
for j=1:n
    if (lsort(j,3) > u/2)
        % don't go beyond u/2
        fprintf("%d break at distance > u/2 \n", j);
        break;
    end
    rx(1) = lsort(j,6);
    rx(2) = lsort(j,7);
    % use error of this estimate if smaller
    if (lsort(j,2) < err)
        err = lsort(j,2);
    end
    [r0, r1] = TOClocate3(pta, rx, ua);
    if (r0 == 1) % found
        LC(1,i) = r1(1);
        LC(2,i) = r1(2);
        ER(i) = err;
        fprintf("%d check ToC OK at %d: %f.%f, err=%f \n", i, j, r1, err);
        return;
    end
end

% no estimate in range, just record least error
ER(i) = err;

% set return to best estimate when further refinement needed
r1 = ptl;
fprintf("LCscan: TOClocate3 not successful return %f %f \n", r1);

end
 
function plotcircs(loc, n, ptf, pta)

global CHGRAD;

figure();
for i=1:n
    rx(1) = loc(i,6);
    rx(2) = loc(i,7);
    viscircles(rx,CHGRAD);
    axis equal;
    if (rx == ptf)
        hold on;
        scatter(rx(1), rx(2), 'r');
    else
        hold on;
        scatter(rx(1), rx(2), 'b', '*');
    end 
end
scatter(pta(1), pta(2), 500, 'X', 'g');
end

    

   
 
    

   