%--------------------------------------------------------------------------
%
%   generateParameter.m
%
%   This script generates the parameter u, and its derivatives, used in the
%   parametric equations of motion primitives.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [T,u,du,ddu,dddu] = generateParameter(ts,ti,tf,qi,qf,dqi,dqf)
    Dq = qf - qi;
    DT = tf - ti;
    % Compute the interpolation coefficients
    a0 = qi;
    a1 = dqi;
    a2 = (3*Dq - (2*dqi+dqf)*DT)/DT^2;
    a3 = (-2*Dq - (dqi+dqf)*DT)/DT^3;
    % Generate the profiles
    T = 0:ts:DT;
    u = a0 + a1*T + a2*T.^2 + a3*T.^3;
    du = 3*a3*T.^2 + 2*a2*T + a1*ones(size(T));
    ddu = 6*a3*T + 2*a2*ones(size(T));
    dddu = 6*a3*ones(size(T));
    T = ti + T; % Consider the initial time
end