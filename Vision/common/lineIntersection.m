%--------------------------------------------------------------------------
%
%   lineIntersection.m
%
%   This script computes the intersection between the lines given as (q,m)
%   pairs.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function p = lineIntersection(q1,m1,q2,m2)
    % Calculate L1 and L2 which minimizes distance and the m1 and m2 points
    assert(abs(norm(m1-m2)) > 1e-10, 'Lines are parallel (any point is an intersection)')
    q = m1'*m2;
    Q = (q1-q2)/(1-q);
    L1 = q*Q'*m2 - Q'*m1;
    p1 = q1 + L1*m1;
    L2 = Q'*m2 - q*Q'*m1;
    p2 = q2 + L2*m2;
    p = mean([p1 p2],2);
end