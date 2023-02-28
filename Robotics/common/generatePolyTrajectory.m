%--------------------------------------------------------------------------
%
%   generatePolyTrajectory.m
%
%   This function generates a polynomial trajectory of the 3rd order, 5th
%   order (if ai and af are provided) or of the 7th order (if ai, af, ji
%   and jf are provided). The arguments are:
%
%       ti,tf: initial and final time
%       qi,qf: initial and final position
%       vi,vf: initial and final velocity
%       ai,af: initial and final acceleration
%       ji,jf: initial and final jerk
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [Q,V,A,J,S] = generatePolyTrajectory(ti, tf, qi, qf, vi, vf, ai, af, ji, jf)
    syms t a0 a1 a2 a3 a4 a5 a6 a7
    
    % Setup the position symbolic equations
    if nargin < 6
        error("Not enough arguments for traj function");
    else
        q = a0 + a1*(t-ti) + a2*(t-ti)^2 + a3*(t-ti)^3;
        if nargin == 8
            q = a0 + a1*(t-ti) + a2*(t-ti)^2 + a3*(t-ti)^3 + a4*(t-ti)^4+ a5*(t-ti)^5;
        elseif nargin == 10
            q = a0 + a1*(t-ti) + a2*(t-ti)^2 + a3*(t-ti)^3 + a4*(t-ti)^4+ a5*(t-ti)^5 + a6*(t-ti)^6+ a7*(t-ti)^7;
        end
    end
    % Compute the symbolic derivatives
    v = diff(q, t);
    a = diff(v, t);
    j = diff(a, t);
    s = diff(j, t);
    
    % Setup the system of equations
    t = ti; eq1 = subs(q) == qi;
    t = ti; eq2 = subs(v) == vi;
    t = tf; eq3 = subs(q) == qf;
    t = tf; eq4 = subs(v) == vf;
    if nargin == 8
        t = ti; eq5 = subs(a) == ai;
        t = tf; eq6 = subs(a) == af;
        [A,B]=equationsToMatrix([eq1,eq2,eq3,eq4,eq5,eq6],[a0,a1,a2,a3,a4,a5]);
    elseif nargin == 10
        t = ti; eq5 = subs(a) == ai;
        t = tf; eq6 = subs(a) == af;
        t = ti; eq7 = subs(j) == ji;
        t = tf; eq8 = subs(j) == jf;
        [A,B]=equationsToMatrix([eq1,eq2,eq3,eq4,eq5,eq6,eq7,eq8],[a0,a1,a2,a3,a4,a5,a6,a7]);
    else
        [A,B]=equationsToMatrix([eq1,eq2,eq3,eq4],[a0,a1,a2,a3]);
    end
    % Solve the system for the coefficients of the polynomial
    x = simplify(linsolve(A,B));
    t = ti:0.001:tf;
    % Use the coefficients to compute the trajectory numerically
    if nargin == 8
        a0 = x(1); a1 = x(2); a2 = x(3); a3 = x(4); a4 = x(5); a5 = x(6);
        Q = subs(q); V = subs(v); A = subs(a); J = subs(j); S = subs(s);
    elseif nargin == 10
        a0 = x(1); a1 = x(2); a2 = x(3); a3 = x(4); a4 = x(5); a5 = x(6); a6 = x(7); a7 = x(8);
        Q = subs(q); V = subs(v); A = subs(a); J = subs(j); S = subs(s);
    else
        a0 = x(1); a1 = x(2); a2 = x(3); a3 = x(4);
        Q = subs(q); V = subs(v); A = subs(a); J = 0*t + subs(j); S = 0*t; % 0*t trick to make the lines show
    end
end
