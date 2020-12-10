clc
close all
clear
t=0:0.01:(2*pi);
q=zeros(7,length(t));
q(6,:)=ones(1,1)*sin(t)*pi;
dq=zeros(7,length(t));
dq(6,:)=ones(1,1)*cos(t)*pi;
a_h=1;
a_r=5;
a_t=10;
k_1=5;
a=0.3;
alpha=1;
x_i=ones(3,1);
%  q=ones(7,1)*sin(t)*pi;
%  dq=ones(7,1)*cos(t)*pi^2;
Delta_x=ones(3,1)*sin(t)*pi;
xe=zeros(3,length(t));
for j=1:length(t)
  [xe(:,j),R]=func_forwardKinematics(q);  
end

xdes=zeros(3,length(t));
xdes=xe-Delta_x;

dxdes=[xdes(:,1), diff(xdes')']/0.01;
ddxdes=[dxdes(:,2), diff(dxdes')']/0.01;

dqr=zeros(7,length(t));
ddqr=zeros(7,length(t));

for k=1:length(t)
    
J = func_getJacobian(q(:,k));
J=J(1:3,:);
j11(k)=J(2,2);
Jplus=J'/(J*J' + eye(3)*0.1);
jplus11(k)=Jplus(1,1);
dj=derivate_jacobian(q(:,k),dq(:,k));
dj=dj(1:3,:);
dj11(k)=dj(2,2);
djplus=derivate_Jplus(J,dj);
djplus11(k)=djplus(1,1);

region=dominant_region(Delta_x(:,k),a_h,a_r,a_t);              % region evaluaion
w=weight_vector(Delta_x(:,k), region, a_r, a_t);               % weighted vector  (10)
A=transition_matrix_A(xe(:,k), xdes(:,k), w , a_r, a_t);            % transition matrix A (7) (dubbi sull'ampiezza della A)
x_d=x_i+w*(xdes(:,k)-x_i);                                     % weighted trajectory (5)
delta_x(:,k)=xe(:,k)-x_d;                                           % weighted position error (4)

M=eye(7)-Jplus*A*J;
M11(k)=M(2,2);
dxe(:,k)=J*q(:,k);
d_Delta_x(:,k)=dxe(:,k)-dxdes(:,k);
dA=derivate_A(Delta_x(:,k), d_Delta_x(:,k), xdes(:,k), dxdes(:,k), a_r, a_t, region);
d_inv_M=derivate_inv_M(M, J, djplus, A, dA, dj);
d_inv_M11(k)=d_inv_M(2,2);
dx_f(:,k)=w*dxdes(:,k)-A*dxdes(:,k);

k_p(k)=pos_stiffness(delta_x(:,k), k_1, a, a_h);
dqr(:,k)=M\(Jplus*dx_f(:,k)-alpha*Jplus*k_p(k)*delta_x(:,k));
ddqr(:,k)=derivate_dqr(M, J, A, delta_x(:,k), d_Delta_x(:,k), a_h, a, k_1, region, q(:,k), dq(:,k), dxdes(:,k), ddxdes(:,k), dxe(:,k), dx_f(:,k), Delta_x(:,k), xdes(:,k), a_r, a_t, w, alpha, k_p(k));
end
figure(1)
plot(t,q(1,:),t,dq(1,:))
title('B q1, R dq1')

figure(2)
plot(t,j11,t,dj11)
title('B j11, R dj11')

figure(3)
plot(t,jplus11,t,djplus11)
title('B j^+11, R dj^+11')

figure(4)
plot(t,M11,t,d_inv_M11)
title('B M11, R dinvM11')

figure(5)
plot(t,dqr(1,:),t,ddqr(1,:))
title('B dqr, R ddqr')