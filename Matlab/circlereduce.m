% circlereduce.m
% disk cover reduction for extended PI
% © 2019 Paul Durham, School of Computer Science, Carleton University
%
function [r0] = circlereduce(c, r, i, xp, yp)
% r0 = 1, circle covered
% r0 = -1, circle not covered
% c = array of circles
% n = count
% r = radius
% i = circle under test
% xp = position of x in c
% yp = position of y in c

n = size(c,1);

angles = [];

ct(1) = c(i,xp);
ct(2) = c(i,yp);

k = 0;
for j=1:n
    if (j ~= i)
        % centres must be <= r from each other
        co(1) = c(j,xp);
        co(2) = c(j,yp);
        %D = [co; ct];
        dc = mydist(co, ct);
        if (dc > r)
            %fprintf("circles %d and %d too far apart: %f\n", i, j, dc);
        else
            % find points of intersection
            [x, y] = circcirc(ct(1),ct(2),r,c(j,xp),c(j,yp),r);
            %[xy] = intersectCircles([ct(1),ct(2),r],[c(j,xp),c(j,yp),r]);
            %x = xy(:,1);
            %y = xy(:,2);
            % intersection has been found
            %fprintf("%d, %d: %f : %f,%f %f,%f\n", i, j, dc, x, y);
            
            % vector between centres
            yc = c(j,yp)-ct(2);
            xc = c(j,xp)-ct(1);
            ac = atan2(yc,xc);
            if (ac < 0)
                ac = ac + pi * 2;
            end
            
            % vectors from centre to intersections
            y1 = y(1)-ct(2);
            x1 = x(1)-ct(1);
            y2 = y(2)-ct(2);
            x2 = x(2)-ct(1);
            
            % dot product
            xyd = x1*x2 + y1*y2;
            
            % normalize - both vectors are radii
            xyn = xyd/(r*r);
            
            % angle of sector
            as = acos(xyn);
            
            %fprintf("xy=%f, ac=%f, as=%f, i=%d, j=%d, k=%d\n", xy, ac, as, i, j, k);
            
            k = k + 1;
            angles(k,1) = ac;
            angles(k,2) = as;
        end
    end
end

if (k < 3)
  % cannot cover with < 3 neighbours
  r0 = -1;
  return;
 end
  
% sort angles
nangles = sortrows(angles);

r0 = 0;
while (r0 == 0)
    [r0, nangles] = sreduce(nangles);
end

if (r0 == 1)
    fprintf("circle %d covered\n", i);
end
end


function [r0, r1] = sreduce(sangles)
% r0 =  0, angles consolidated
% r0 = -1, angles could not be consolidated
% r0 =  1, angle covers whole circle
% r1 = sangles after consolidation (only if r0 = 0)

% assume fail
r0 = -1;
r1 = [];

n = size(sangles, 1);
if (n == 1)
    % only one angle
    if (sangles(1, 2) >= pi*2)
        % circle covered
        r0 = 1;
    end
    return;
end

% consolidate first 2 angles
a1 = sangles(1,1);
t1 = sangles(1,2);
a2 = sangles(2,1);
t2 = sangles(2,2);

% check if sector 1 is entirely within sector 2
if ((a1-t1/2 >= a2-t2/2) && (a1+t1/2 <= a2+t2/2))
  fprintf("sector 1 is inside sector 2\n");
  sangles(1,:) = [];
  r0 = 0;
  r1 = sangles;
  return;
end

% check if sector 2 is entirely within sector 1
if ((a1-t1/2 <= a2-t2/2) && (a1+t1/2 >= a2+t2/2))
  fprintf("sector 2 is inside sector 1\n");
  sangles(2,:) = [];
  r0 = 0;
  r1 = sangles;
  return;
end

% overlap if top of 1st sector is beyond bottom of 2nd sector
ovl = (a1 + t1/2) - (a2 - t2/2); 
% do sectors 1 and 2 overlap
if (ovl >= 0)
    % new radius is halfway between extremes of old sectors
    anew = ((a2 + t2/2) + (a1 - t1/2))/2;
    % new span is sum of old spans minus overlap
    tnew = t1 + t2 - ovl;
    sangles(1,1) = anew;
    sangles(1,2) = tnew;
    sangles(2,:) = [];
    r0 = 0;
    r1 = sangles;
end

end

function [r0] = mydist(a, b)
x2 = (a(1)-b(1))^2;
y2 = (a(2)-b(2))^2;
r0 = (x2+y2)^.5;
end