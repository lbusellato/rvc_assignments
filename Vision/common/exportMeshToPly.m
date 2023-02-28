%--------------------------------------------------------------------------
%
%   exportMeshToPly.m
%
%   This script converts a set of vertices, faces, vertex colors to .ply 
%   format .
%
%--------------------------------------------------------------------------
function exportMeshToPly(vertices, faces, vertex_color, name)
    if max(max(vertex_color)) <= 1.0
        vertex_color = vertex_color.*256;
    end
    if size(vertex_color,2) == 1
        vertex_color = repmat(vertex_color,1,3);
    end
    vertex_color = uint8(vertex_color);
    fidply = fopen([name '.ply'], 'w');
    fprintf(fidply, 'ply\n');
    fprintf(fidply, 'format ascii 1.0\n');
    fprintf(fidply, 'element vertex %d\n', size(vertices,1));
    fprintf(fidply, 'property float x\n');
    fprintf(fidply, 'property float y\n');
    fprintf(fidply, 'property float z\n');
    fprintf(fidply, 'property uchar red\n');
    fprintf(fidply, 'property uchar green\n');
    fprintf(fidply, 'property uchar blue\n');
    fprintf(fidply, 'element face %d\n', size(faces,1));
    fprintf(fidply, 'property list uchar int vertex_index\n');
    fprintf(fidply, 'end_header\n');
    for i=1:size(vertices,1)
        fprintf(fidply, '%f %f %f %d %d %d\n', ...
            vertices(i,1), vertices(i,2), vertices(i,3), ...
            vertex_color(i,1), vertex_color(i,2), vertex_color(i,3));
    end
    for i=1:size(faces,1)
        fprintf(fidply, '3 %d %d %d\n', ...
            faces(i,1)-1, faces(i,2)-1, faces(i,3)-1);
    end
    fclose(fidply);
end