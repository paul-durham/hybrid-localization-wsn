% Intersect.m
% solve perpendicular intersection equation for original PI
% © 2019 Paul Durham, School of Computer Science, Carleton University
function [r1] = Intersect(pa, pb, pc, idxab, idxbc, ns)
% compute position based on sides 1-2 and 2-3
    %M = [p2-p1, 0, 0, 0, 0, p3 - p2];
    M = zeros(2,4);
    M(1,1) = pb(1)-pa(1);
    M(1,2) = pb(2)-pa(2);
    M(2,3) = pc(1)-pb(1);
    M(2,4) = pc(2)-pb(2);

    Q = zeros(2);
    Q(1,1) = M(1,1);
    Q(1,2) = M(1,2);
    Q(2,1) = M(2,3);
    Q(2,2) = M(2,4);

    % compute coordinates of perpendicular
    N = zeros(4,1);
    N(1) = (pa(1)*(ns-idxab)+pb(1)*(idxab-1))/(ns-1);
    N(2) = (pa(2)*(ns-idxab)+pb(2)*(idxab-1))/(ns-1);
    N(3) = (pb(1)*(ns-idxbc)+pc(1)*(idxbc-1))/(ns-1);
    N(4) = (pb(2)*(ns-idxbc)+pc(2)*(idxbc-1))/(ns-1);

    MN = M*N;

    X = Q \ MN;

    r1 = X';
end