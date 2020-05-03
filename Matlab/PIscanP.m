% PIscanP.m
% scan for extended perpendicular intersection
% edge to edge equilateral or isosceles tiling
% © 2019 Paul Durham, School of Computer Science, Carleton University
function [r0] = PIscanP(fs, ml, mr, mb, mt, p1, theta, delta, hint, n, PP, u, ht, std, sf, sr, mvp, ua, ntraces)
%This function does initial scan of field
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% p1 = starting point
% delta = distance between points
% hint = hello interval, number of deltas between hellos
% theta = starting angle in units of pi/3
% n = number of nodes
% PP = array of nodes
% u = unit disk radius
% ht = UAV hight in metres
% std = log normal fading standard deviation
% sf = scale factor metres/unit distance
% sr = sample rate samples/metre
% mvp = moving average point count
% ua = uav
% ntraces = number of traces

global ptfoundup;
global ptfounddn;
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

iu = zeros(2,10);
iuc = 0;
%id = zeros(10);
%idc = 0;
il = zeros(10);
ilc = 0;
ir = zeros(10);
irc = 0;

% wavelength of network frequency
wl = 300 / 2400;
% wavelength of charging frequency
wc = 300 / 910;

% starting point
pt = p1;

% previous point
prv = pt;

% bounce count
bcnt = 1;

% keep going until every node has an up trace and a down trace
% each iteration represents UAV going from one side to another
j = 0;
done = 0;
while (done ==0)
    % reset bounces
    bt = 0;
    bb = 0;
    bl = 0;
    br = 0;
    
    
    % move UAV another increment
    angle = theta * pi / 3;
    xinc = delta*cos(angle);
    yinc = delta*sin(angle);
    pt = pt + [xinc, yinc];
    if (mod(j,hint) == 1)
        UAVmove(pt, ua);
    end
    
    % check for bounce
    if (pt(2) >= mt)
        % top bounce
        bt = 1;
        pt(2) = mt;
        iuc = iuc+1;
        iu(:,iuc) = pt;
    end
    if (pt(2) <= mb)
        % bottom bounce
        bb = 1;
        pt(2) = mb;
        iuc = iuc+1;
        iu(:,iuc) = pt;
        
    end
    if (pt(1) >= mr)
        br = 1;
        pt(1) = mr;
    end
    if (pt(1) <= ml)
        bl = 1;
        pt(1) = ml;
    end
    
    % if bounce end trace on all nodes
    if (br || bl || bt || bb)
        UAVmove(pt, ua);
        for i=1:n
            % get WSNnode
            o = matlabJava.WSNnode.getnode(i);
            % end trace
            rc = o.endRSSI(false);
            if (rc > 0)
                % a viable trace for this pass has completed
                %%if (theta == 1 || theta == 4)
                if (theta >= 0 && theta < 3/2 || theta > 3 && theta < 4.5)
                    % positive slope
                    ptfoundup(i) = 1;
                else
                    % negative slope
                    ptfounddn(i) = 1;
                end
            end
        end
        
        % draw line upon bounce
        hold on;
        plot([prv(1), pt(1)], [prv(2), pt(2)]);
        hold off;
        prv = pt;
        
        bcnt = bcnt + 1;
    end
    
    j = j + 1;

    % check RSSI on every "hint" moves
    if (mod(j,hint) == 1)
        % found count
        f = 0;
        % scan active flags
        scanactive = zeros(n);
        for i=1:n
            % get WSNnode
            o = matlabJava.WSNnode.getnode(i);
            % get node coordinates
            pta(1) = PP(1,i);
            pta(2) = PP(2,i);
            D = [pt; pta];
            r = pdist(D);
            if (r<=u)
                % node is within radius
                % show scan active
                scanactive(i) = 1;
                
                f = f + 1;
                % distance to actual node in metres
                r = r * 100;
                % calculate rss in dbuW using 2 plane formula
                rss = 10*log(1000000 * 10.76 * r^4 / ((ht^2 + r^2))^3 * (wl^2)/(16*pi^2));
                
                % add log normal fading
                lnf = std * randn * r/(r + ht);
                rss = rss + lnf;
                
                rc = o.addRSSI(pt(1), pt(2), rss);
                if (rc == 1 && (theta == 1 || theta == 4))
                    % make found node green
                    hold on;
                    scatter(pta(1), pta(2), 'g');
                    hold off;
                end
            else
                % out of radius - end trace if active
                if (scanactive(i)== 1)
                    scanactive(i) = 0;
                    rc = o.endRSSI(false);
                    if (rc == -1)
                        fprintf("node %d: %f %f, uav %f %f, rc = %d: trace too short %f, %f\n", i, pta(1), pta(2), pt(1), pt(2), rc, r, u);
                    else
                        if (rc > 0)
                            % a viable trace for this pass has completed
                            %if (theta == 1 || theta == 4)
                            if (theta >= 0 && theta < 3/2 || theta > 3 && theta < 4.5)
                                % positive slope
                                ptfoundup(i) = 1;
                            else
                                % negative slope
                                ptfounddn(i) = 1;
                            end
                        end
                    end
                end
            end
        end
    end
    if (f == 0)
        %fprintf("wtf \n");
    end
    %fprintf("point found = %d\n", f);
    
    % check for boundary hit
    if (br || bl)
        % hit x boundary
        fprintf("x boundary at %f %f, theta=%d \n", pt, theta);
        theta = 3 - theta;
        if (theta < 0)
            theta = theta + 6;
        end
    else
        if (bt || bb)
            % hit y boundary
            fprintf("y boundary at %f %f, theta=%d \n", pt, theta);
            theta = 6 - theta;
        end
    end
    
    %
    if (br || bl || bt || bb)
        
        j = 0;
        k = 0;
        for i=1:n
            if (ptfoundup(i) > 0)
                j = j + 1;
            end
            if (ptfounddn(i) > 0)
                k = k + 1;
            end
        end
        fprintf("found up = %d, down = %d\n", j, k);
        if (bcnt >= 10)
            fprintf("bcnt = %d\n", bcnt);
        end
        if ((j == n && k == n) && (bcnt > ntraces))
            matlabJava.WSNnode.dumpNodes();
            done = 1;
            fprintf("all found\n");
        end
    end
end

% return last location
r0 = pt;

for i=1:n
    % get WSNnode
    o = matlabJava.WSNnode.getnode(i);
    
    pta(1) = PP(1,i);
    pta(2) = PP(2,i);
    
    % now compute location based on best 2 traces
    %nu = o.getBestUp();
    %nd = o.getBestDown();
    
    % get the longest up and down traces
    nu = o.getUpDown(1);
    nd = o.getUpDown(-1);
    
    o.dumpNode();
    
    if (nu>0 && nd>0)
        
        [rv, r123, ca] = GetIntersect(o, nu, nd, mvp);
        
        err = pdist2(r123, pta, 'euclidean');
        
        if (err > .10)
            fprintf("node %d, tr=%d,%d rv=%d, cos=%f, lc=%f,%f error=%f\n", i, nu, nd, rv, ca, r123, err);
        end
        
        LC(1,i) = r123(1);
        LC(2,i) = r123(2);
        
        UP(i) = nu;
        DN(i) = nd;
        
        TC(i) = o.getTraceCnt();
        
        ER(i) = err;
    end
end
end

function rc0 = inELT(P, P1, P2, P3, s)
% is P inside the equilateral triangle
% s = side length
% centre of triangle
c = (P1+P2+P3)/3;
% height of triangle
h = s*(3^.5)/2;

% if point is in triangle it will not be more than
% altitude of p
a = abs(P(2)-P1(2));
% distance from centre
d = abs(P(1)-(P1(1)+P3(1))/2);

if (d >  s/2  )
    % it's outside horizontal or vertical bounds
    rc0 = 0;
else
    % max altitude to be in triangle
    m = h * (1-d*2/s);
    if (a <= m)
        rc0 = 1;
    else
        rc0 = 0;
    end
end
end
    
    function t = intriangle(P, P1, P2, P3)
    s = det([P1-P2;P3-P1]);
    t = s*det([P3-P;P2-P3])>=0 & s*det([P1-P;P3-P1])>=0 & s*det([P2-P;P1-P2])>=0;
    end
    
 
    

   