% number of points
nx = size(PP);
n = nx(2);

% unit disk radius
u = .2;
% noise factor
nf = .2;

% height in metres
global HT;
HT = 1.0;

% wavelength of network frequency
global WLN;
WLN = 300 / 2400;

% wavelength of charging frequency
global WLC;
WLC = 300 / 910;

% UAV move count
global uavmvct;
uavmvct = 0;

% UAV total distance
global uavdist;
uavdist = 0;

% UAV previous point
global uavprev;
uavprev = [-.5, -.5];

x1 = "call ESDP"

% perform localization
[PP,dd,m,r,nf, Xiter] = ESDP(PP,5,n,u,nf, 10);

x2 = "ESDP returns"

m = 0;
% dd is upper triangle matrix of distances
% edge exists if distance <= u
for i=2:n
    for j=1:i-1
        if ((dd(j,i)~= 0) && (dd(j,i) <= u))
            m = m + 1;
        end
    end
end

% m is the number of edges
% average degree is m * 2 / n
fprintf("Node count=%d, u=%d, edge count=%d, average degree = %d\n", n, u, m/2, m/n);

% define UAV icon
x = [-.005  , .005, .015, -.015];
y = [-.01  , -.01, .01, .01];
g = hgtransform;
uav = patch('XData',x,'YData',y,'FaceColor','yellow','Parent',g);
uav.DisplayName = '$';

%axis equal
%xlim([-10 10])
%ylim([-10 10])
%pt1 = [-3 -4 0];

% start UAV from here
pt1 = [-.5 -.5 0];

%pt2 = [5 2 0];
%pt2 = [6 8 0];

%plot all edges
%for i = 1:n
%    for j = i+1:n
%        if (dd(i,j) ~= 0)
%            %plot([PP(1,i),PP(1,j)],[PP(2,i),PP(2,j)]);
%        end
%    end
%end

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

% number of steps
s = 30;

% received signal strength
%rss = zeros(n,s);

% locate count
lc = 0;
% scan count
sc = 0;
% iterate through localized coordinates
for i = 1:n-4
    %nb(i, :)
    % localized coordinate
    ptl =  [Xiter(1,i), Xiter(2,i)];
    % actual coordinate
    pta = [PP(1,i), PP(2,i)] ;
    % distance to neighbour
    %b = nb(i,1);
    %a = i;
    %dist = getdistance(i, mynb, dd);
    
    % b goes from -.5 to .5. Calculate r in metres
    %r = b * 1000;
    
    % if (a>b)
    %    dist = dd(b,a);
    %else
    %    dist = dd(a,b);
    %end
    
    % locate flag
    lo = 0;
    j = 0;
    % starting point
    pt = [-0.5, -0.5];
    % distance to localized coordinate
    dl = pdist([pt; ptl], 'euclidean');
    t = 0;
    d = 0;
    vec = ptl-pt;
    rot = 0;
    inc = .1;
    
    %while (d < dl)
    while (0)
        % move inc of unit radius at a time
        d = d + u*inc;
        % (pt2-pt1)/dl is unit vector pointing to LC
        % current position
        pt = pt + inc * u * vec/dl;
        % distance to actual node
        r = pdist([pt;pta], 'euclidean');
        j = j + 1;
        if (r > u)
            uav.FaceColor = 'yellow';
            rss(i,j) = 0;
        else
            uav.FaceColor = 'green';
            lo = 1;
        end
        
        g.Matrix = makehgtform('translate',pt);
        drawnow
    end
    if (lo > 0)
        lc = lc + 1;
    end
    
    % start with triangle side = unit disk radius
    s = u;
    
    % keep searching until inscribed circle smaller than peak radius
    pr = (2 * HT^2) ^.5; % peak radius in metres
    ps = pr * 6 / (3^.5)/100; % side of circumtriangle in 100 metres
    
    uav.FaceColor = 'green';
    %var = 4;
    var = 4;
    i = 0;
    mva = 5;
    r0 = 0;
    while (r0 ~= 1 && s > ps*2)
        [r0, r1] = PIscan(pta, ptl, i, s, var, mva, g);
        s = s/2;
        var = var / 2;
        ptl = r1;
        i = i + 1;
    end
    
    % now ensure we are in the charging area
    uav.FaceColor = 'blue';
    [r0, r1] = TOCscan(pta, ptl, g);
    ptl = r1;
    if (r0 == 1)
        sc = sc + 1;
    end
    
    % minimize the charging area
    uav.FaceColor = 'red';
    TOClocate(pta, ptl, g);
    
end

fprintf("mva count = %f, scans needed = %f \n", mva, sc);

function dist = getdistance(a,b, dds)

if (b>a)
    dist = dds(b,a);
else
    if (a<b)
        dist = dds(a,b);
    else
        dist = 0;
    end
end

end

