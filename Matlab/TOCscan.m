% TOCscan.m
% check if in feasible area, reduce if true, spiral if not
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
function [r0, r1] = TOCscan(pta, ptl, ua)
%This function first checks if UAV is in charging area, i.e. p >= MINCHG
% if so, it returns
% if not, it executes a spiral checking power until it is in area
%
% r0 = 0 no scan needed
%   = 1 found after scan
%   = -1 scan terminated without find
% r1 = location found
%
% note coordinates are given in range [-0.5, 0.5]
% corresponds to -50 to 50 metres
% pta = actual location of node
% ptl = localized estimate for node
% ua = uav

pt = ptl;

% EIRP of charger
global EIRP;

% minimum charging power in dBm
global MINCHG;

% wavelength of charging frequency
global WLC;

% height in metres
global HT;

% charging radius
global CHGRAD;

% rx gain = 3.28
gr = 3.28;

% distance to actual node in metres
r = pdist([pt;pta], 'euclidean') * 100;

% calculate charging power im dbm in vertical direction
%rss0 = 10*log10(1000 * 10.76 * (wl/(4*pi))^2 * 1/(ht^2));
%rss = 10*log10(1000 * 10.76 * (1-r^2/(ht^2 + r^2))^2 * (wl/(4*pi))^2 * 1/(ht^2 + r^2));
rss = pchg(EIRP, gr, WLC, HT, r);
%fprintf("TOCscan r = %f, rss = %f dbm, rss0 = %f dbm \n", r, rss, rss0);

% search increments
% uav will not be more than CHGRAD m away
hinc = CHGRAD * 1.4;
vinc = CHGRAD * 1.4;

% uav will move in a spiral
% first two sides are 1 unit
% next two sides are 2 units, etc.
i = 0; % total count
ln = 1; % current side length
bc = 0;
% assume no scan needed
r0 = 0;
%
while(rss<MINCHG)
    % need to figure out the next move
    % odd side length is right or up
    if (mod(ln,2) == 1)
        if (i < bc+ln)
            % this is move right
            pt = pt + [hinc, 0];
        else
            % this is move up
            pt = pt + [0, vinc];
            
        end
    else
        % even side length is left or down
        if (i < bc+ln)
            % this is move left
            pt = pt + [-hinc, 0];
        else
            % this is move down
            pt = pt + [0, -vinc];
        end
    end
    
    i = i + 1;
    
    % increment side length after last move up
    if(i == bc + 2*ln)
        ln = ln + 1;
        bc = i;
    end
    
    % move uav
    UAVmove(pt, ua);
    %ua.Matrix = makehgtform('translate',[pt(1) pt(2), 0]);
    %drawnow
    
    % distance to actual node in metres
    r = pdist([pt;pta], 'euclidean') * 100;
    
    rss = pchg(EIRP, gr, WLC, HT, r);
    
    fprintf("TOCscan r = %f, rss = %f dbm \n", r, rss);
    r0 = 1;
end

r1 = pt;

end