%--------------------------------------------------------------------------
%
%   genenrateSmoothSplines.m
%
%   This script implements the computation of the smooth cubic splines that 
%   approximate the points (qk,tk) given the smoothing factor mu and the
%   weight vector w.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [p,v,a,j,t,m] = generateSmoothSplines(qk, tk, w, mu)
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
    C = zeros(n+1,n+1);
    C(1,1) = -6/T(1,1); C(1,2) = 6/T(1,1);
    C(end,end) = -6/T(1,end); C(end,end-1) = 6/T(1,end);
    for k = 2:n % Construct the known terms matrix
        C(k,k) = -(6/T(k-1)+6/T(k));
        C(k,k-1) = 6/T(k-1);
        C(k,k+1) = 6/T(k);
    end
    for k = 1:n+1 % Compute inv(W)
        if w(1,k)~=inf
            w(1,k) = 1/w(1,k);
        else
            w(1,k) = 0;
        end
    end
    W = diag(w); % nv(W)
    lambda = (1-mu)/(6*mu);
    dds = (C*qk')'/(A+lambda*C*W*C'); 
    s = (qk' - lambda*W*C'*dds')'; 
    t = []; p = []; v = []; a = []; j = []; m = [1];
    for k = 1:n
        % Trajectory computation
        dt = tk(1,k):0.001:tk(1,k+1);
        t = [t dt];
        q = @(dt) (s(1,k+1)/T(1,k)-T(1,k)*dds(1,k+1)/6)*(dt-tk(1,k)) + ...
            (s(1,k)/T(1,k)-T(1,k)*dds(1,k)/6)*(tk(1,k+1)-dt) + ...
            dds(1,k)*(tk(1,k+1)-dt).^3/(6*T(1,k)) + ...
            dds(1,k+1)*(dt-tk(1,k)).^3/(6*T(1,k));
        p = [p q(dt)];
        dq = @(dt) dds(1,k+1)*(dt-tk(1,k)).^2/(2*T(1,k)) - dds(1,k)*(tk(1,k+1)-dt).^2/(2*T(1,k)) + ...
            (s(1,k+1)-s(1,k))/T(1,k)-T(1,k)*(dds(1,k+1)-dds(1,k))/6;
        v = [v dq(dt)];
        ddq = @(dt) (dds(1,k+1)*(dt-tk(1,k))+dds(1,k)*(tk(1,k+1)-dt))/T(1,k);
        a = [a ddq(dt)];
        dddq = @(dt) (dds(1,k+1)-dds(1,k))/T(1,k) + 0*dt;
        j = [j dddq(dt)];
        m = [m size(t,2)];
    end
    m = [m size(t,2)];
end