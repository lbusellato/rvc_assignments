%--------------------------------------------------------------------------
%
%   genenrateCubicSplinesAcc.m
%
%   This script implements the computation of cubic splines that
%   interpolate the points (qk,tk) based on the accelerations with given
%   initial/final velocities.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

function [p,v,a,j,t,m] = generateCubicSplinesAcc(qk, tk, dqk)
    % Construction of the A matrix
    n = size(qk,2)-1;
    T = []; % Time intervals Tk = t(k)-t(k+1)
    for k = 1:n
        T(1,k) = tk(1,k+1)-tk(1,k);
    end
    A = zeros(n+1,n+1);
    A(1,1) = 2*T(1,1); A(1,2) = T(1,1);
    A(end,end) = 2*T(1,end); A(end,end-1) = T(1,end);
    for k = 2:n % Construct the tridiagonal matrix
        A(k,k) = 2*(T(k-1)+T(k));
        A(k,k-1) = T(k-1);
        A(k,k+1) = T(k);
    end
    c = 6*((qk(1,2)-qk(1,1))/T(1,1)-dqk(1,1));
    for k = 2:n % Construct the known terms vector
        c = [c 6*((qk(1,k+1)-qk(1,k))/T(1,k)-(qk(1,k)-qk(1,k-1))/T(1,k-1))];
    end
    c = [c 6*(dqk(1,2)-(qk(1,2)-qk(1,1))/T(1,1))];
    ddqk = thomas(A,c); % Faster than ddqk = inv(A)*c
    t = []; p = []; v = []; a = []; j = []; m = [1];
    for k = 1:n
        % Computation of the a coefficients
        a0 = qk(1,k);
        a1 = (qk(1,k+1)-qk(1,k))/T(1,k)-(T(1,k)*(ddqk(1,k+1)+2*ddqk(1,k))/6);
        a2 = 0.5*ddqk(1,k);
        a3 = (ddqk(1,k+1)-ddqk(1,k))/(6*T(1,k));
        % Trajectory computation
        dt = tk(1,k):0.001:tk(1,k+1);
        t = [t dt];
        q = @(dt) a3*(dt-tk(1,k)).^3 + a2*(dt-tk(1,k)).^2 + a1*(dt-tk(1,k)) + a0;
        p = [p q(dt)];
        dq = @(dt) 3*a3*(dt-tk(1,k)).^2 + 2*a2*(dt-tk(1,k)) + a1;
        v = [v dq(dt)];
        ddq = @(dt) 6*a3*(dt-tk(1,k)) + 2*a2;
        a = [a ddq(dt)];
        dddq = @(dt) 6*a3 + 0*dt;
        j = [j dddq(dt)];
        m = [m size(t,2)];
    end
    m = [m size(t,2)];
end