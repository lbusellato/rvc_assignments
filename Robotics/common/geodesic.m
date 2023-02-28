%--------------------------------------------------------------------------
%
%   geodesic.m
%
%   This script computes the geodesic curve between points P1 and P2 given
%   the sphere centre P0 and the step du for the parameter of the
%   parametrization. The script also computes the Frenet frame associated
%   to each trajectory point.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [G, t, n, b] = geodesic(P1, P2, P0, du)
    
    % Compute the radius
    R0 = norm(P1 - P0);
    % Compute the radius vector for the two points
    R1 = (P1 - P0) / R0;
    R2 = (P2 - P0) / R0;
    % Compute the direction of the circle connecting P1 and P2
    R = cross(R1, R2);
    R = R/norm(R);
    % Collect three directions
    Rt = [R1 cross(R, R1) R];
    % Compute rotations
    Ri = [cross(R, -R1) R -R1];
    Rf = [cross(R, -R2) R -R2];
    Rif = Ri'*Rf;
    % Angle from P1 to P2
    uf = acos((trace(Rif) - 1) / 2);
    % Compute the points from P1 to P2
    u = 0:du:uf;
    G = Rt * [R0 * cos(u);
              R0 * sin(u);
              zeros(size(u))] + P0;
    G = G';
    % Compute the Frenet frame associated to each point
    scale = -3; % Just to improve visualization
    t = Rt * [-R0*sin(u); R0*cos(u); zeros(1,size(u,2))];
    t = scale*t./norm(t);
    n = Rt * [-R0*cos(u); -R0*sin(u); zeros(1,size(u,2))];
    n = scale*n./norm(n);
    b = cross(t,n);
%     Sr = [    0 -r(3)  r(2); 
%          r(3)     0 -r(1); 
%          -r(2)  r(1)     0];
%     v1 = reshape(1-cos(u),1,1,[]);
%     v2 = reshape(sin(u),1,1,[]);
%     v3 = reshape(cos(u),1,1,[]);
%     Ru = r*r'.*v1 + Sr.*v2 + eye(3).*v3;
%     F = zeros(size(Ru));
%     for i=1:size(Ru,3)
%         F(:,:,i) = -Ri*Ru(:,:,i);
%     end 
end