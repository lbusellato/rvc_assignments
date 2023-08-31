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
    % Solution to the system of equations obtained with the two parametric
    % forms of the lines and the orthogonality constraint of the segment
    % connecting them to the lines themselves
    alpha = (q1-q2)'*m1;
    beta = m1'*m1;
    gamma = m2'*m1;
    psi = (q1-q2)'*m2;
    k = m2'*m2;
    % Solutions
    muA = (psi*gamma-alpha*k)/(beta*k-gamma*gamma);
    muB = (psi+muA*gamma)/k;
    % The "intersection" is actually the midpoint
    pa = q1 + muA*m1;
    pb = q2 + muB*m2;
    p = (pa + pb) / 2;    
end