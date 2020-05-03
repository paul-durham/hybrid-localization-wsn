% pitile.m
% original perpendicular intersection algorithm
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
% number of points
nx = size(PP);
n = nx(2);

% unit disk radius
u = .2;

% height in metres
global HT;
HT = 1.0;

% wavelength of network frequency
global WLN;
WLN = 300 / 2400;

% wavelength of charging frequency
global WLC;
WLC = 300 / 910;

% EIRP of charger
global EIRP;
% charging radius
global CHGRAD;

% comment/uncomment the following for desired charger EIRP
% charging radius for -9.5 dBm, EIRP = 4 W
EIRP = 4.0;
CHGRAD = 0.0182;

%charging radius for -19.5 dBm, EIRP = 40 W
%EIRP = 40.0;
%CHGRAD = 0.0288;

%charging radius for -19.5 dBm, EIRP = 80 W
%EIRP = 80.0;
%CHGRAD = 0.03274;

%charging radius for -19.5 dBm, EIRP = 160 W
%EIRP = 160.0;
%CHGRAD = 0.037;

% minimum charging power in dBm
global MINCHG;
MINCHG = -9.5;

global ptlocated;
ptlocated = 0;
global ptrejected;
ptrejected = 0;
global ptfound;
ptfound = zeros(1,n);

% UAV move count
global uavmvct;
uavmvct = 0;

% UAV total distance
global uavdist;
uavdist = 0;

% UAV previous point
global uavprev;
uavprev = [-.5, -.5];

% localized coordinates
global LC;
LC = zeros(2, n);

% error
global ER;
ER = zeros(1, n);

% error from 1st scan
%global E1;
%E1 = zeros(1, n);

% within 1m count
global C1;
C1 = 0;

% within 5m count
global C5;
C5 = 0;

% within 10m count
global C10;
C10 = 0;

% within footprint after initial PI
global F1;
F1 = 0;

% within footprint after refined PI
global F2;
F2 = 0;

% noise factor
nf = .2;

x1 = "call generateD"

% perform localization
%[PP,dd,m,r,nf, Xiter] = ESDP(PP,4,n,u,nf, 5);
[PP,dd] = generateD(PP,4,n,u,nf, 5);

plot(PP(1,:), PP(2,:), '*');
xlim([-.6,.6]);
ylim([-.6,.6]);

x2 = "generateD returns"

% define UAV icon
x = [-.005  , .005, .015, -.015];
y = [-.01  , -.01, .01, .01];
g = hgtransform;
uav = patch('XData',x,'YData',y,'FaceColor','yellow','Parent',g);
uav.DisplayName = '$';

% want to do triangle scan of field
dir = 1;
% standard deviation of log normal RSSI
std = 4;
% moving average count
mva = 5;
% start with triangle side = unit disk radius
s = u;
% starting point
p1(1) = (.5 + .5 * s) * dir * (-1);
p1(2) = -.5;

% traverse y axis
while(p1(2)<=.5)
    % traverse x axis
    while((dir == 1 && p1(1)<.5+s/2) || dir == -1 && p1(1)>-.5-s/2)
        
        % define triangle
        p2d = [dir * s/2, s*(3^.5)/2];
        p2 = p1 + p2d;
        p3d = dir * [s, 0];
        p3 = p1 + p3d;
        
        % right side up triangle
        PIscan1(p1, p2, p3, n, PP, dir, s, std, mva, g, true);
        
        % draw line from p1 to p2
        hold on;
        plot([p1(1), p2(1)], [p1(2), p2(2)]);
        hold off;
        
        % draw line from p2 to p3
        hold on;
        plot([p2(1), p3(1)], [p2(2), p3(2)]);
        hold off;
        
        % upside down triangle
        q1 = p2;
        q2 = p3;
        q3 = q1 + dir * [s,0];
        PIscan1(q1, q2, q3, n, PP, dir, s, std, mva, g, false);
        
        % move right or left
        p1 = p1 + dir * [s,0];
    end
    
    % move up vertically
    p1 = p1 + [0, s*(3^.5)/2];
    dir = -dir;
    
    if(p1(2)<=.5)
        % draw line from p3 to p1
        %hold on;
        %plot([p3(1), p1(1)], [p3(2), p1(2)]);
        %hold off;
        UAVmove(p1, g);
    end
end

% save scan distance
uavscan = uavdist;
uavdist = 0;

% neighbour array
nb = zeros(n,n);
nc = zeros(n,1);

% fill neighbour array
for i = 1:n
    for j = i+1:n
        if (dd(i,j) > 0)
            nc(i)= nc(i)+ 1;
            nc(j)= nc(j)+ 1;
            nb(i,nc(i)) = j;
            nb(j,nc(j)) = i;
            
        end
    end
end

% locate count
lc = 0;
% scan count
sc = 0;
% iterate through localized coordinates
for j = 1:n
    
    % accuracy counts
    if (ER(j) < .01)
        C1 = C1 + 1;
    else if (ER(j) < .05)
            C5 = C5 + 1;
        else if (ER(j) < .10)
                C10 = C10 + 1;
            end
        end
    end
    
    % localized coordinate
    ptl = LC(:,j).';
    % actual coordinate
    pta = PP(:,j).';
    
    UAVmove(ptl, g);
    
    % in charging area?
    [r0, r1] = TOClocate3(pta, ptl, g);
    if (r0 == 1)
        fprintf("F1 hit at %d\n", j);
        F1 = F1 + 1;
    else
        % refined PI
        % start with triangle side = 1/2 unit disk radius
        s = u/2;
        
        % keep searching until inscribed circle smaller than peak radius
        pr = (2 * HT^2) ^.5; % peak radius in metres
        ps = pr * 6 / (3^.5)/100; % side of circumtriangle in 100 metres
        
        uav.FaceColor = 'blue';
        
        i = 1;
        mva = 5;
        r0 = 0;
        while (r0 ~= 1 && s > ps*2)
            [r0, r1] = PIscan(pta, ptl, i, s, std, mva, g);
            s = s/2;
            std = std / 2;
            ptl = r1;
            i = i + 1;
        end
        
        % now ensure we are in the charging area
        uav.FaceColor = 'yellow';
        [r0, r1] = TOCscan(pta, ptl, g);
        ptl = r1;
        if (r0 == 1)
            % ptl was not in charging area
            sc = sc + 1;
            fprintf("scan at %d\n", j);
        else
            % ptl was in charging area
            F2 = F2 + 1;
        end
        
        % minimize the charging area
        uav.FaceColor = 'red';
        TOClocate3(pta, ptl, g);
        
    end
end

fprintf("mva count = %f, scans needed = %f \n C1,C5,C10=%d,%d,%d\n", mva, sc, C1, C5, C10);
fprintf("F1 = %d, F2 = %d, sc = %d, uavscan = %f, uavdist = %f\n", F1, F2, sc, uavscan, uavdist);

% output to files
f = fopen("ptileStat", "a");
fprintf(f, "%d, %d, %d, %f, %f\n", F1, F2, sc, uavscan, uavdist);

g = fopen("ptileErr", "a");
for i=1:n
    fprintf(g, "%f\n", ER(i));
end





