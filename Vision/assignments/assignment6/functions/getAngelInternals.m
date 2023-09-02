function [K, R, t] = getAngelInternals(name)
    fid=fopen(name, 'r');
    
    %Fixed structure:
    string=fgetl(fid);
    string=fgetl(fid);
    %Third line contains intrinsic:
    intrinsics=fgetl(fid);
    C = textscan(intrinsics,'%s');
    out=C{1};
    fx=str2num(out{7}(5:end-1));
    fy=str2num(out{8}(5:end-1));
    cx=str2num(out{9}(5:end-1));
    cy=str2num(out{10}(5:end-1));
    
    K=[fx,    0,        cx;
        0,   fy,        cy;
        0,   0,          1];
end

