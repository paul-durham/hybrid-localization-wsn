% pong.m
% extended perpendicular intersection algorithm
% edge to edge equilateral or isosceles tiling
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
javaclasspath('.')
javaclasspath('-dynamic')
%javaclasspath('-static')


global ptlocated;
ptlocated = 0;
global ptrejected;
ptrejected = 0;

% angle of up trace = 60 degrees
global UPANGLE;
UPANGLE = pi/3;

% angle of up trace = 120 degrees
global DNANGLE;
DNANGLE = pi/1.5;

% number of points
nx = size(PP);
n = nx(2);

global ptfoundup;
ptfoundup = zeros(1,n);
global ptfounddn;
ptfounddn = zeros(1,n);

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

% localized coordinate found flags
global LF;
LF = zeros(1, n);

% localization count
global LT;
LT = 0;

% most recent up trace used for LC
global UP;
UP = zeros(1,n);

% most recent down trace used for LC
global DN;
DN = zeros(1,n);

% trace count for most recent localization
global TC;
TC = zeros(1,n);

% error
global ER;
ER = zeros(1,n);

% within 1m count
global C1;
C1 = 0;

% within 5m count
global C5;
C5 = 0;

% within 10m count
global C10;
C10 = 0;

% noise factor
nf = .15;

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

%charging radius for -9.5 dBm, EIRP = 40 W
%EIRP = 40.0;
%CHGRAD = 0.0288

%EIRP = 80.0;
%CHGRAD = 0.03274;

%EIRP = 160.0;
%CHGRAD = 0.037;

% maximum number of estimates to test
maxest = 100;

% minimum charging power in dBm
global MINCHG;
MINCHG = -9.5;

% transmission radius
global U;
U = 0.2;

% within footprint after initial PI
global F1;
F1 = 0;

% within footprint after refined PI
global F2;
F2 = 0;

% spiral required
global F3;
F3 = 0;

global LC1;
LC1 = 0;

% field size
r3 = 3.0 ^.5;
u = .2;
L = 7.0*u;
H = L*r3/2.0;

% comment or uncomment the following for equilateral vs. isosceles
% starting angle for UAV
theta = 1; % equilateral
%theta = asin(H/((H^2+(u/2)^2)^.5)) / pi * 3; % isosceles

% number of traces in initial scan
tracecnt = 18; % equilateral
%tracecnt = 12; % isosceles

% start with triangle side = 1/2 unit disk radius
pistart = u/2; % equilateral
%pistart = u; % isosceles

x1 = "call generateD"

% generate unit disk graph
%[PP,dd] = generateD(PP,4,n,u,nf, 5);
plot(PP(1,:), PP(2,:), '*');

% bottom margin
mb = -H/2;
% top margin
mt = H/2 ;
% right margin
mr = L/2-u/2;
% left margin
ml = -L/2+u/2;

xlim([ml,mr]);
ylim([mb,mt]);

x2 = "generateD returns"

% creates WSNnode instances
% nodes will be added to static array in WSNnode
matlabJava.WSNnode.initNodes();
for i=1:n
    o = matlabJava.WSNnode(i, PP(1,i), PP(2,i), 20);
end
methods(o)
%matlabJava.WSNnode.dumpNodes();

% define UAV icon
x = [-.005  , .005, .015, -.015];
y = [-.01  , -.01, .01, .01];
g = hgtransform;
uav = patch('XData',x,'YData',y,'FaceColor','yellow','Parent',g);
uav.DisplayName = '$';

% starting point of UAV
p1 = [ml, mb];
% standard deviation of log normal RSSI
std = 4;
% moving average count for RSSI
mva = 5;
% hc = hello count, number of hellos per u distance
hc = 25;
% hint = hello interval, number of moves per hello
hint = 10;
% each move travels "delta". Every "hint" moves a hello is sent
% there will be hc * hint moves over distance u
delta = u / hc / hint;
sf = 100;
sr = 5;
mvp = 5;
fs = 1.0;

[pt] = PIscanP(fs, ml, mr, mb, mt, p1, theta, delta, hint, n, PP, u, HT, std, sf, sr, mvp, g, tracecnt);

% following code not currently used
% neighbour array
%nb = zeros(n,n);
%nc = zeros(n,1);

% fill neighbour array
%for i = 1:n
%    for j = i+1:n
%        if (dd(i,j) > 0)
%            nc(i)= nc(i)+ 1;
%            nc(j)= nc(j)+ 1;
%            nb(i,nc(i)) = j;
%            nb(j,nc(j)) = i;
%            
%        end
%    end
%end
% end code not currently used

uavscan = uavdist;
uavdist = 0;

% locate count
lc = 0;
% scan count
sc = 0;
% iterate through nodes
i = 1;
while (i<= n)
    %while( LT < n)
    % try any node not yet localized
    if (LF(i) == 0)
        % localized coordinate
        ptl(1) = LC(1,i);
        ptl(2) = LC(2,i);
        
        % actual coordinate
        pta(1) = PP(1,i);
        pta(2) = PP(2,i);
        
        % move UAV to localized coordinate
        [r0, r1] = LCscan(i, ml, mr, mb, mt, pt, theta, delta, hint, n, PP, u, HT, std, sf, sr, mvp, g, maxest);
        
        % test if coordinate is within charging footprint
        pt = r1;
        if (r0 > 0)
            LF(i) = 1;
            LT = LT + 1;
            F1 = F1 + 1;
            fprintf("%d in charging footprint, count=%d\n", i, LT);
        else
            fprintf("%d try refined PI\n", i);
            
            % perform refined PI
            % localized coordinate
            ptl = pt;
            
            % starting triangle side
            s = pistart;
            
            % keep searching until inscribed circle smaller than peak radius
            pr = (2 * HT^2) ^.5; % peak radius in metres
            ps = pr * 6 / (3^.5)/100; % side of circumtriangle in 100 metres
            
            uav.FaceColor = 'blue';
            
            j = 1;
            mva = 5;
            r0 = 0;
            while (r0 ~= 1 && s > ps*2)
                [r0, r1] = PIscan(pta, ptl, j, s, std, mva, g);
                s = s/2;
                std = std / 2;
                ptl = r1;
                j = j + 1;
            end
            if (r0 == 1)
               fprintf("%d refined PI found charging footprint\n", i);
               F2 = F2 + 1;
            else
               % PIscan did not locate charging footprint
               % just spiral until found
                  [r0, r1] = TOCscan(pta, ptl, g);
                  if (r0 == 1)
                      uav.FaceColor = 'red';
                      [r0, r1] = TOClocate3(pta, r1, g);
                      F3 = F3 + 1;
                  end
            end
            uav.FaceColor = 'green';
        end
    end
    i = i + 1;
    %   if (i > n)
    %       i = 1;
    %   end
end

for i=1:n
    % accuracy counts
    if (ER(i) < .01)
        C1 = C1 + 1;
    else if (ER(i) < .05)
            C5 = C5 + 1;
        else if (ER(i) < .10)
                C10 = C10 + 1;
            end
        end
    end
end

fprintf("mva count = %f, in charging footprint = %d \n C1,5,10=%d,%d,%d\n", mva, LT, C1, C5, C10);
fprintf("F1 = %d, F2 = %d, F3 = %d, uavscan = %f, uavdist = %f\n", F1, F2, F3, uavscan, uavdist);

% output to files
f = fopen("pongStat", "a");
fprintf(f, "%d, %d, %d, %f, %f\n", F1, F2, F3, uavscan, uavdist);

g = fopen("pongErr", "a");
for i=1:n
    fprintf(g, "%f\n", ER(i));
end

