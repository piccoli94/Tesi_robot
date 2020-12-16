function tau=region_controller(q,dq,xdes,dxdes,ddxdes,f_h,theta,a_h,a_r,a_t,x_i,k_1,a,alpha,K_s,L,beta,sigma,w_s)
% q joints position
% dq joints velocity
% xde position reference
% dxde velocity reference
% a_h H-DR radius
% a_r R-DR radius
% a_t TS-SS radius
% x_i time-invariant reference position
% k1,a positive constant in position dependent stiffness
% alpha sliding vector constant
% K_s sliding gain matrix
% theta dynamic parameter vector
% L positive non singular square matrix gain
% f_h interaction force
% beta,sigma interaction region defining constants [0,pi/2]
% w_s saturation term
[xe,R]=func_forwardKinematics(q,7);                           % forward kinematics
Delta_x=xe-xdes;                                          % position error
region=dominant_region(Delta_x,a_h,a_r,a_t);              % region evaluaion
w=weight_vector(Delta_x, region, a_r, a_t);               % weighted vector  (10)
A=transition_matrix_A(xe, xdes, w , a_r, a_t);            % transition matrix A (7) 
x_d=x_i+w*(xdes-x_i);                                     % weighted trajectory (5)
delta_x=xe-x_d;                                           % weighted position error (4)
k_p=pos_stiffness(delta_x, k_1, a, a_h)                   % position dependent stiffness (17)
J= func_getJacobian(q,7)                                  % Jacobian   (da testare)
J=J(1:3,:);                                               % Jacobian Rows number must be equal to  Operative space dimension 
dxe=J*dq;                                                 % operation space velocity
d_Delta_x=dxe-dxdes;                                      % velocity error
M=eye(7)-pinv(J)*A*J;                       % Modifier Matrix (6)
dx_f=w*dxdes-A*dxdes;                                     % virtual position reference vector (8a)
dqr=M\(pinv(J)*dx_f-alpha*pinv(J)*k_p*delta_x);           % derivate of virtual joint reference vector (8b)
s=dq-dqr;                                                 % sliding vector (8)
ddqr=derivate_dqr();
Y_d=regres_matrix(q, dq, dqr, ddqr);                      %da fare (trovare modello dinamico)  %regressor matrix
theta_cap=theta_estimate(theta, Y_d, L)                   % estimated dynamic parameter vector (9)

s_x=J*s;                                                  % composite variable (20)
interact_region=force_diff(f_h, s_x, beta, sigma);        % interaction region selctor
mu_s=saturation_func(s_x, w_s);                           % saturation function (23a)
mu_x=transition_smooth_func(region, a_r, a_h, Delta_x);    % smooth transition function from HD and RD (23)
c_x=interaction_tem(interact_region, beta, sigma, f_h, s_x); %interaction term (22)
C_x=interaction_controller(mu_s, mu_x, c_x, f_h, region); % term of interaction controller (21)

tau_m_a=-M'*J'*k_p*delta_x;                               % stiffnes controller (3a)
tau_m_b=-K_s*s;                                           % sliding controller (3b)
tau_m_c=Y_d*theta_cap;                                    % adaptive controller (3c)

tau_m=tau_m_a + tau_m_b + tau_m_c;                        % motion-related controller (3)
tau_i=J'*C_x;                                             % interaction-related controller (19)
tau=tau_m+tau_i;                                          % robot controller (2)