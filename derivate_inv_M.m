function d_inv_M=derivate_inv_M(M, j, dJplus, A, dA, dj)

dM=-dJplus * A * j - pinv(j) * dA * j - pinv(j) * A * dj;
d_inv_M=M\dM/M;