%--------------------------------------------------------------------------
%
%   thomas.m
%
%   This script uses Thomas' algorithm to solve the system Ax=d with A
%   tridiagonal.
%
%   Author: Lorenzo Busellato, VR472249, 2023
%
%--------------------------------------------------------------------------
function x = thomas(A, d)    
    a = []; b = []; c = [];
    n = size(A, 2);
    for i = 1:n % Cycle all rows
        b = [b A(i,i)]; % Diagonal terms
        if i < size(A,1)
            c = [c A(i,i+1)]; % "Upper" diagonal terms
        end
        if i > 1
            a = [a A(i,i-1)]; % "Lower" diagonal terms
        end
    end
    % Forward elimination
    for k = 1:size(a,2)
        m = a(1,k)/b(1,k);
        b(1,k+1) = b(1,k+1) - m*c(1,k);
        d(1,k+1) = d(1,k+1) - m*d(1,k);
    end
    x = zeros(1,n); x(1,end) = d(1,n)/b(1,n);
    % Backward substitution
    for k = n-1:-1:1
        x(1,k) = (d(1,k)-c(1,k)*x(1,k+1))/b(1,k);
    end
end