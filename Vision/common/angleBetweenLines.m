%--------------------------------------------------------------------------
%
%   angleBetweenLines.m
%
%   This script computes the angle between the given lines.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function a = angleBetweenLines(m1, m2)
    a = acosd((m1'*m2)/(norm(m1)*norm(m2)));
end