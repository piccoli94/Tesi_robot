function ddqr=derivate_dqr(M, j, A, delta_x, a_h, a, k_1, region, q, dq, dxdes, ddxdes, dxe, d_xf, Delta_x, xdes, a_r, a_t, w, alpha, k_p)

dA=derivate_A(Delta_x, xdes, dxdes, a_r, a_t, region);
d_delta_x = derivate_delta_x(dxe, d_xf, A);
dj=derivate_jacobian(q,dq); dj_part=dj.part; dj_int=dj.int;
djplus=derivate_Jplus(j,dj_part);
d_inv_M=derivate_inv_M(M, j, djplus, A, dA, dj_int);
dk_p=derivate_kp(delta_x, d_delta_x, a_h, a, k_1, region);
dw=derivate_w(Delta_x, d_delta_x, region, a_r, a_t);
dd_xf=derivate_dxf(w, dw, dxdes, ddxdes, A, dA);

dot1= d_inv_M * (pinv(j) * d_xf - alpha * pinv(j) * k_p * delta_x);
dot2=M \ (djplus * d_xf + pinv(j) * dd_xf);
dot3=M \ (- alpha * djplus * k_p * delta_x - alpha * pinv(j) * dk_p * delta_x - alpha * pinv(j) * k_p * d_delta_x);

ddqr=dot1+dot2+dot3;