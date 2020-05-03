% TOClocate3.m
% reduce charger feasible area to 90% power
% © 2019 Paul Durham, School of Computer Science, Carleton University
function [r0, r1] = TOClocate3(pta, ptl, ua)
%This function locates node by reducing feasible area using TOC trisection
% r0 = 0 can't locate because not in charging footprint
% r0 = 1 located and charged
% r1 = ToC located coordinate
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% pta = actual location of node
% ptl = localized estimate for node
% ua = uav

% charging radius
global CHGRAD;

global HT;
global WLC;
global EIRP;
global MINCHG;

pt = ptl;

% if i even feasible area is a square which will reduce to rectangle
% if i odd feasible area is a rectangle which will reduce to a square
i = 0; % total count

% rx gain = 3.28
gr = 3.28;

% move uav
UAVmove(pt, ua);

% try current location
r = pdist([pt;pta], 'euclidean') * 100;
rss = pchg(EIRP, gr, WLC, HT, r);

% default returns
r0 = 0;
r1 = pt;

% if rss < MINCHG not within charging footprint
if (rss < MINCHG)
    fprintf("%f %f not in footprint, r=%f, rss=%f\n", r1, r, rss);
    return
end

pmax = pchg(EIRP, gr, WLC, HT, 0);

fprintf("%f %f in footprint, r=%f, rss=%f, pmax=%f\n", r1, r, rss, pmax);

% side of square is 2 x radius
s = CHGRAD*2;

% increment is 1/3 of the way from centre
% i.e. 1/3 of a side
hinc = s/3;
vinc = s/3;

% within footprint, try for pmax-2dB
r0 = 1;
while(rss < pmax-2)
        % try right
        ptr = pt + [hinc, 0];
        r = pdist([ptr;pta], 'euclidean') * 100;
        UAVmove(ptr, ua);
        rsr = pchg(EIRP, gr, WLC, HT, r);
        
        % pchg is concave in r
        % if right is better than centre there is no need to go left
        if (rsr > rss)
            pt = ptr; % move right
            rss = rsr;
        else
            % try left
            ptl = pt + [-hinc, 0];
            r = pdist([ptl;pta], 'euclidean') * 100;
            UAVmove(ptl, ua);
            rsl = pchg(EIRP, gr, WLC, HT, r);
            
            fprintf("TOClocate3 rss = %f, rsr = %f, rsl = %f \n", rss, rsr, rsl);
            
            % is left better than centre
            if (rsl > rss)
                pt = ptl; % move left
                rss = rsl;
            end
        end
        
        % try up
        ptu = pt + [0, vinc];
        r = pdist([ptu;pta], 'euclidean') * 100;
        UAVmove(ptu, ua);
        rsu = pchg(EIRP, gr, WLC, HT, r);
        
        % if up is better than centre there is no need to go down
        if (rsu > rss)
            pt = ptu; % move up
            rss = rsu;
        else
            % try down
            ptd = pt + [0, -vinc];
            r = pdist([ptd;pta], 'euclidean') * 100;
            UAVmove(ptd, ua);
            rsd = pchg(EIRP, gr, WLC, HT, r);
            
            fprintf("TOClocate3 rss = %f, rsu = %f, rsd = %f \n", rss, rsu, rsd);
            
            % is down better than centre
            if (rsd > rss)
                pt = ptd; % move down
                rss = rsd;
            end
        end
        hinc = hinc/3;
        vinc = vinc/3;
end
