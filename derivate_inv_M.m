function d_inv_M=derivate_inv_M(M, j, dJplus, A, dA, dj)
Jplus=j'/(j*j' + eye(3)*0.1);
dM=-dJplus * A * j - Jplus * dA * j - Jplus * A * dj;
d_inv_M=-M\dM/M;