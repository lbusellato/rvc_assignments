%--------------------------------------------------------------------------
%
%   generateCubicSplines.m
%
%   This script generates the cubic splines that interpolate the (qk,tk)
%   points, with the possibility of specifying point velocities dqk,
%   initial final velocities [dq0 dqn] with the intermediate velocity
%   computation done with Euler or with Thomas' algorithm.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [p,v,a,j,t,m] = generateCubicSplines(qk, tk, dqk, th)
    % Check if we need to compute the waypoint velocities   
    n = size(qk,2) - 1;
    if size(dqk,2) == 2 % Only initial and final velocities provided
        if ~th % Implement Euler approximation
            vk = [];
            for k = 2:n
                dq = qk(1,k)-qk(1,k-1); % Tk
                dt = tk(1,k)-tk(1,k-1); % dqk
                vk = [vk dq/dt];
            end
            vk = [dqk(1) vk dqk(end)];
            dqk = [dqk(1)]; 
            for k = 2:n
                if sign(vk(1,k))~=sign(vk(1,k+1))
                    dqk = [dqk 0];
                else
                    dqk = [dqk 0.5*(vk(1,k)+vk(1,k+1))];
                end
            end
            dqk = [dqk dqk(end)];
        else % Compute tridiagonal matrix and solve with Thomas algorithm
            T = [];
            for k = 1:n % Compute the time intervals
                T = [T tk(1,k+1)-tk(1,k)];
            end
            A = zeros(n-1,n-1);
            A(1,1) = 2*(T(1)+T(2)); A(end,end) = 2*(T(end-1)+T(end));
            A(1,2) = T(1); A(2,1) = T(2); A(end,end-1) = T(end);
            for k = 2:n-2 % Construct the tridiagonal matrix
                A(k,k) = 2*(T(k)+T(k+1));
                A(k,k-1) = T(k+1);
                A(k,k+1) = T(k);
            end
            d = [];
            for k = 1:n-1 % Compute the known terms
                dk = 3*(T(k+1)*(qk(k+1)-qk(k))/T(k)+T(k)*(qk(k+2)-qk(k+1))/T(k+1));
                d = [d dk];
            end
            d(1) = d(1)-dqk(1)*T(1); d(end) = d(end)-dqk(end)*T(n-2);
            dqk = [dqk(1) thomas(A,d) dqk(2)];
        end
    end
    t = []; p = []; v = []; a = []; j = []; m = [1];
    for k = 1:n
        % Computation of the a coefficients
        a0 = qk(1,k);
        a1 = dqk(1,k);
        Tk = 1/(tk(1,k+1)-tk(1,k));
        dq = qk(1,k+1)-qk(1,k);
        a2 = Tk*(3*dq*Tk-2*dqk(1,k)-dqk(k+1));
        a3 = Tk^2*(-2*dq*Tk+dqk(1,k)+dqk(k+1));
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