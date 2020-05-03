% IntersectP.m
% solve perpendicular intersection equation for extended PI
% © 2019 Paul Durham, School of Computer Science, Carleton University
% 
function [r1] = IntersectP(pa, pb, pc, pd, idxab, idxcd, nsab, nscd)
% compute position based on sides 1-2 and 3-4
    %M = [p2-p1, 0, 0, 0, 0, p4 - p3];
    M = zeros(2,4);
    M(1,1) = pb(1)-pa(1);
    M(1,2) = pb(2)-pa(2);
    M(2,3) = pd(1)-pc(1);
    M(2,4) = pd(2)-pc(2);

    Q = zeros(2);
    Q(1,1) = M(1,1);
    Q(1,2) = M(1,2);
    Q(2,1) = M(2,3);
    Q(2,2) = M(2,4);

    % compute coordinates of perpendicular
    N = zeros(4,1);
    N(1) = (pb(1)*(nsab-idxab)+pa(1)*(idxab-1))/(nsab-1);
    N(2) = (pb(2)*(nsab-idxab)+pa(2)*(idxab-1))/(nsab-1);
    N(3) = (pd(1)*(nscd-idxcd)+pc(1)*(idxcd-1))/(nscd-1);
    N(4) = (pd(2)*(nscd-idxcd)+pc(2)*(idxcd-1))/(nscd-1);

    MN = M*N;

    X = Q \ MN;

    r1 = X.';
end