% PIscan1.m
% scan for original perpendicular intersection algorithm
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
function [r0, r1] = PIscan1(p1, p2, p3, n, PP, dir, s, std, mvp, ua, uam)
%This function does initial scan of field
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% p1, p2, p3 = vertices of triang
% n = number of nodes
% pts = starting point
% PP = array of nodes
% dir = direction 1 = increasing x, -1 decreasing x
% s = triangle side length
% std = log normal standard deviation
% mvp = moving average point count
% ua = uav
% uam = 1 move the UAV

global ptlocated;
global ptfound;
global LC;
% this is the 1st error
global ER;
global WLN;

% uav height in metres
ht = 1.0;

% height of triangle
h = s*(3^.5)/2;

% report nodes in triangle
% j = index of point
for j=1:n
    % for each point in triangle scan position
    pta(1) = PP(1,j);
    pta(2) = PP(2,j);
    
    rc0 = intriangle(pta, p1, p2, p3);
    if (rc0 == true)
        %fprintf("in triangle %f ... %f: %f %f\n", p1(1), p3(1), pta);
        %hold on;
        %plot(pta(1), pta(2), 'x');
        %hold off;
        ptlocated = ptlocated + 1;
        if(ptfound(j) == 1)
            fprintf("problem %f \n", j);
        else
            ptfound(j) = 1;
            
            % edge p1 - p2
            
            % vector from p1 to p2
            vec = p2-p1;
            
            % ns is number of samples per side of triangle
            ns = 21;
            rss12 = zeros(1, ns);
            
            for i=0:ns-1
                % current position
                pt = p1 + i * vec/(ns-1);
                % distance to actual node in metres
                r = pdist([pt;pta], 'euclidean') * 100;
                %d12(i+1) = r;
                % calculate rss in dbuW using 2 plane formula
                rss = 10*log(1000000 * 10.76 * r^4 / ((ht^2 + r^2))^3 * (WLN^2)/(16*pi^2));
                
                % add log normal fading 
                lnf = std * randn * r/(r + ht);
                rss12(i+1) = rss + lnf;
 
                % move uav
                if (uam)
                    UAVmove(pt, ua);
                end
            end
            
            % take moving mean
            rss12m = movmean(rss12, mvp);
            
            % edge p2 - p3
            
            % vector from p2 to p3
            vec = p3-p2;
            rss23 = zeros(1, ns);
            
            for i=0:ns-1
                % current position
                pt = p2 + i * vec/(ns-1);
                % distance to actual node in metres
                r = pdist([pt;pta], 'euclidean') * 100;
                % calculate rss in dbuW using 2 plane formula
                rss = 10*log(1000000 * 10.76 * r^4 / ((ht^2 + r^2))^3 * (WLN/(4*pi))^2);
                
                % add log normal fading 
                lnf = std * randn * r/(r + ht);
                rss23(i+1) = rss + lnf;
                
                % move uav
                if(uam)
                    UAVmove(pt, ua);
                end
            end
            
            % only want to show UAV move for first scan
            uam = 0;
            
            % take moving mean
            rss23m = movmean(rss23, mvp);
            
            % find max value of rss12m, rss23m
            % exclude first and last elements
            [max12, idx12] = max(rss12m);
            [max23, idx23] = max(rss23m);
            
            % compute position based on sides 1-2 and 2-3
            r123 = Intersect(p1, p2, p3, idx12, idx23, ns);
            err = pdist2(r123, pta, 'euclidean');
            
            LC(1,j) = r123(1);
            LC(2,j) = r123(2);
            
            err = pdist2(r123, pta);
            ER(j) = err;
            
            sr = 3^.5*s/6;
            
            fprintf("pta = %f %f, X123 = %f %f, s=%f, sr=%f, err = %f\n", pta, r123, s, sr, err);
        end
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
    
 
    

   