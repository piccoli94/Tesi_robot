clc
close all
clear
load('Ref')
%parametri
t=0:0.001:length(xm)*0.001-0.001;
a_h=1;
a_r=2.5;
a_t=4;
k_1=1;
a=3;
alpha=10;
x_i=0.5*ones(3,1);
beta=pi/4;
sigma=pi/2;
w_s=20;
%movimento giunti

%  q1=t*0;
%  q2=-t*2;
%  q3=t*0;
%  q4=-t*2;
%  q5=t*0;
%  q6=-t*2;
%  q7=t*0;
 %velocità giunti
%  dq1=[0,diff(q1)]/0.01;
%  dq2=[0,diff(q2)]/0.01;
%  dq3=[0,diff(q3)]/0.01;
%  dq4=[0,diff(q4)]/0.01;
%  dq5=[0,diff(q5)]/0.01;
%  dq6=[0,diff(q6)]/0.01;
%  dq7=[0,diff(q7)]/0.01;
 
% q=[q1;q2;q3;q4;q5;q6;q7];
% dq=[dq1;dq2;dq3;dq4;dq5;dq6;dq7];

%errore che va ad aumentare
Delta_x=ones(3,1)*t/2;

% posizione end effector
% xe=zeros(3,length(t));
% for j=1:length(t)
%   [xe(:,j),R]=func_forwardKinematics(q(:,j));  
% end

%riferimento
xdes=zeros(3,length(t));
xdes=xm-Delta_x;
dxdes=dxm-ones(3,length(t))*1/2;
ddxdes=ddxm;

f_h=0.3*[sin(t/2);sin(t/2-pi/4);sin(t/2-pi/2)];

dqr=zeros(7,length(t));
ddqr=zeros(7,length(t));
xe=xm;
dxe=dxm;
ddxe=ddxm;

%% Iterazioni
for k=1:length(t)
    
j = func_getJacobian(q(:,k));
j=j(1:3,:);

jplus=J_pseudoinverse(j);

region=dominant_region(Delta_x(:,k),a_h,a_r,a_t);              % region evaluaion

w=weight_vector(Delta_x(:,k), region, a_r, a_t);               % weighted vector  (10)

A=transition_matrix_A(xe(:,k), xdes(:,k), w , a_r, a_t);            % transition matrix A (7) (dubbi sull'ampiezza della A)

x_d=x_i+w*(xdes(:,k)-x_i);                                     % weighted trajectory (5)

delta_x(:,k)=xe(:,k)-x_d;                                           % weighted position error (4)

M=eye(7)-jplus*A*j;

d_xf(:,k)=w*dxdes(:,k)-A*dxdes(:,k);

k_p(k)=pos_stiffness(delta_x(:,k), k_1, a, a_h, region);

dqr(:,k)=M\(jplus*d_xf(:,k)-alpha*jplus*k_p(k)*delta_x(:,k));


%% Derivate
 %dxe(:,k)=j*dq(:,k);

dj=derivate_jacobian(q(:,k),dq(:,k));
dj=dj(1:3,:);

djplus=derivate_Jplus(j,dj);

d_Delta_x=dxe(:,k)-dxdes(:,k);

dA=derivate_A(Delta_x(:,k), d_Delta_x, xdes(:,k), dxdes(:,k), a_r, a_t, region);

d_delta_x = derivate_delta_x(dxe(:,k), d_xf(:,k), A);

d_inv_M=derivate_inv_M(M, j, djplus, A, dA, dj);

dk_p=derivate_kp(delta_x(:,k), d_delta_x, a_h, a, k_1, region);

dw=derivate_w(Delta_x(:,k), d_Delta_x, region, a_r, a_t);

dd_xf=derivate_dxf(w, dw, dxdes(:,k), ddxdes(:,k), A, dA);

ddqr=derivate_dqr(M,d_inv_M, delta_x(:,k),d_delta_x, jplus,djplus, d_xf(:,k), dd_xf ,alpha, k_p(k), dk_p);
%% parte sulla forza 
s=dq(:,k)-dqr(:,k);
s_x=j*s;
interact_region=force_diff(f_h(:,k), s_x, beta, sigma);        % interaction region selctor
mu_s=saturation_func(s_x, w_s);                           % saturation function (23a)
mu_x=transition_smooth_func(region, a_r, a_h, Delta_x(:,k));    % smooth transition function from HD and RD (23)
c_x=interaction_tem(interact_region, beta, sigma, f_h(:,k), s_x); %interaction term (22)
C_x=interaction_controller(mu_s, mu_x, c_x, f_h(:,k), region); % term of interaction controller (21)



%%
%%valori da plottare
num=3; norm_x(k)=norm(Delta_x(:,k));
xe1(k)=xe(num,k); dxe1(k)=dxe(num,k);
j11(k)=j(num,num); dj11(k)=dj(num,num);
jplus11(k)=jplus(num,num); djplus11(k)=djplus(num,num); 
w1(k)=w;dw1(k)=dw;
a11(k)=A(num,num); da11(k)=dA(num,num);
delta1(k)=delta_x(num,k); ddelta1(k)=d_delta_x(num); 
in=inv(M); m11(k)=in(num,num); dm11(k)=d_inv_M(num,num);
kp(k)=k_p(k); dkp(k)=dk_p;
ddxf1(k)=dd_xf(num); dxf1(k)=d_xf(num,k);
qrdot1(k)=dqr(num,k); qrdotdot1(k)=ddqr(num);
sx(:,k)=s_x;
end
figure(1)
plot(t,xe(1,:),t,dxe(1,:))
title('B xe, R dxe')

figure(2)
plot(t,j11,t,dj11)
title('B j11, R dj11')

figure(3)
plot(t,jplus11,t,djplus11)
title('B j^+11, R dj^+11')

figure(4)
plot(t,w1,t,dw1)
title('B w, R dw')

figure(5)
plot(t,a11,t,da11)
title('B A, R dA')

figure(6)
plot(t,delta1,t,ddelta1)
title('B deltax, R ddeltax')

figure(7)
plot(t,m11,t,dm11)
title('B m11, R dm11')

figure(8)
plot(t,kp,t,dkp)
title('B kp, R dkp')

figure(9)
plot(t,dxf1,t,ddxf1)
title('B dxf, R ddxf')

figure(10)
plot(t,qrdot1,t,qrdotdot1)
title('B dqr, R ddqr')

figure(11)
plot3(f_h(1,:),f_h(2,:),f_h(3,:));
hold on
plot3(sx(1,:),sx(2,:),sx(3,:));
hold off
title('B xdes, R dxdes')

