%--------------------------------------------------------------------------
%
%   generateMesh.m 
% 
%   This script generates a set of 3D vertices based on
%   the range data contained in depthImg, a set of triangular faces
%   between them and a set of vertex colors based on the intensity values
%   of rgbImg. cameraParams is a vector [fu fv u0 v0], where fu=f*ku,
%   fv=f*kv are the products of the focus length f and the focus
%   lenghts in pixels, ku and kv, u0 and v0 are the coordinates of the
%   principal point. depthThreshold sets the maximum depth value that is
%   considered as a valid reading. 3D cloud of points from range image, mesh
%   reconstruction.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function [vertices, faces, color] = generateMesh(rgbImg, depthImg, cameraParams, depthThreshold)
    if size(cameraParams,2) ~= 4 % Check that we have everything we need
        error('Wrong number of camera parameters, check again.')
    end
    % Extract internal parameters
    fu = cameraParams(1,1);
    fv = cameraParams(1,2);
    u0 = cameraParams(1,3);
    v0 = cameraParams(1,4);  
    [U, V] = size(depthImg);
    [u, v] = meshgrid(1:V,1:U);
    % Mask to filter out points too close or too far away
    mask = 0 < depthImg & depthImg < depthThreshold;
    % Reconstruct the 3D cloud of points
    Z = double(depthImg(mask));
    X = (u(mask)-u0).*Z/fu;
    Y = (v(mask)-v0).*Z/fv;
    vertices = [ X Y Z ];
    % Use the original RGB values to set vertex colors
    R = double(rgbImg(:,:,1));
    G = double(rgbImg(:,:,2));
    B = double(rgbImg(:,:,3));
    color = [R(mask) G(mask) B(mask)]/255;
    % Create a neighbor index map
    ind = reshape(1:U*V, U, V);
    near_ind = ind(mask); % Mask the indexes actually present in the mesh
    indexes = zeros(U,V);
    for i = 1:V
        for j = 1:U
            if mask(j,i) ~= 0
                indexes(j,i) = find(near_ind == sub2ind([U V], j, i));
            end
        end
    end
    count = 1;
    faces = zeros(2*U*V,3);
    % Cycle vertex by vertex and look for the triangles in the 2x2 neighborhood
    for i=1:V-1 
        for j=1:U-1  
            % Checks the orientation of the faces ( |\| or |/|)
            rev = false;
            if mask(j+1,i) && mask(j+1,i+1) && mask(j,i+1)
                % Bottom left, bottom right, upper right
                f = [indexes(j+1,i) indexes(j+1,i+1), indexes(j,i+1)];
                faces(count,:) = f;
                count = count + 1;
                rev = true;
            end
            if mask(j,i)
                if mask(j+1,i)
                    if mask(j,i+1)
                        % Upper left, bottom left, upper right   
                        f = [indexes(j,i) indexes(j+1,i), indexes(j,i+1)];
                        faces(count,:) = f;
                        count = count + 1;
                    else
                        if ~rev && mask(j+1,i+1)
                            % Upper left, bottom left, bottom right    
                            f = [indexes(j,i) indexes(j+1,i), indexes(j+1,i+1)];
                            faces(count,:) = f;
                            count = count + 1;
                        end
                    end
                else
                    if ~rev && mask(j,i+1) && mask(j+1,i+1)
                        % Upper left, bottom right, upper right    
                        f = [indexes(j,i) indexes(j+1,i+1), indexes(j,i+1)];
                        faces(count,:) = f;
                        count = count + 1;
                    end
                end
            end
        end
    end
    % Cull the unused part of the matrix
    faces = faces(1:(count-1), :);
end