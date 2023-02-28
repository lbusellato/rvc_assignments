%--------------------------------------------------------------------------
%
%   multipointTrapezoidal.m
%
%   This script implements the computation of a multipoint trapezoidal
%   velocity trajectory, given a set of position waypoints, the
%   initial/final velocity, the maximum velocity/acceleration. The script
%   also implements the possibility of husing an heuristic for choosing the
%   velocity at each point.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------

function [q,v,a,t,markers] = generateTrapezoidal(qks, dqi, dqf, vm, am, cb)
    sz = size(qks, 2);
    tks = zeros(1,sz);
    dqk = zeros(1,sz); 
    if cb % Implement velocity selection rule
        for i = 2:sz-1
            if sign(qks(1,i)-qks(1,i-1))==sign(qks(1,i+1)-qks(1,i))
                dqk(1,i) = sign(qks(1,i)-qks(1,i-1))*vm;
            else
                dqk(1,i) = 0;
            end
        end
    end
    q = []; v = []; a = []; t = []; markers = [1];
    % Iteratively construct the multipoint trajectory
    for k=1:sz-1
        [qk,vk,ak,tk] = generateTrapezoidal(tks(1,k),tks(1,k+1),qks(1,k),qks(1,k+1),nan,nan,nan,dqk(1,k),dqk(1,k+1),am,vm);
        if size(tk, 2) ~= 0
            tks(1,k+1) = tk(size(tk, 2)); 
        else
            tks(1,k+1) = tks(1,k);
        end
        q = [q qk]; v = [v vk]; a = [a ak]; t = [t tk]; 
        markers = [markers size(t,2)];
    end
end