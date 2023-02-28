%--------------------------------------------------------------------------
%
%   rectilinear.m
%
%   This script generates the position, velocity, acceleration and jerk 
%   between profiles for a rectilinear motion primitive between pi and pf,
%   parametrized wrt u.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [q,dq,ddq,dddq] = rectilinear(u,du,ddu,dddu,qi,qf)
    q = qi + u.*(qf-qi);
    dq = du.*(qf-qi);
    ddq = ddu.*(qf-qi);
    dddq = dddu.*(qf-qi);
end