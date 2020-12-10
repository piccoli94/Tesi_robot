clear all
clc
close all
a_h=1;
a_r=5;
a_t=10;
k_1=5;
a=0.3;
t=0:0.01:2;
Delta_x=ones(3,1)*6*sin((0:0.01:2)*pi);
xdes=ones(3,1)*1*sin((0:0.01:2)*pi);
dxdes=zeros(3,length(t));
dxdes=[0,diff(xdes(1,:)); 
       0,diff(xdes(2,:)); 
       0,diff(xdes(3,:))]; 
ddxdes=[0,diff(dxdes(1,:)); 
       0,diff(dxdes(2,:)); 
       0,diff(dxdes(3,:))];    
xe=xdes+Delta_x;
x_i=zeros(3,length(Delta_x));
dxe=[diff(xe(1,:)), 0;
    diff(xe(2,:)) , 0;
    diff(xe(3,:)) , 0];
d_Delta_x=dxe-dxdes;

for k=1:length(Delta_x)
region=dominant_region(Delta_x(:,k),a_h,a_r,a_t);
if (strcmp(region,'SS-SR'))
    x_i(:,k)=xe(:,k);
end
w(k)=weight_vector(Delta_x(:,k), region, a_r, a_t); 
norm_delta(k)=norm(Delta_x(:,k));
x_d(:,k)=x_i(:,k)+w(k)*(xdes(:,k)-x_i(:,k));
delta_x(:,k)=xe(:,k)-x_d(:,k);
A=transition_matrix_A(xe(:,k),xdes(:,k),w(k),a_r,a_t);
dx_f(:,k)=w(k)*dxdes(:,k)-A*dxdes(:,k);
d_delta_x(:,k) = derivate_delta_x(dxe(:,k), dx_f(:,k), A);
dw(k)=derivate_w(Delta_x(:,k), d_delta_x(:,k), region, a_r, a_t);
k_p(k)=pos_stiffness(delta_x(:,k), k_1, a, a_h);
dk_p(k)=derivate_kp(delta_x(:,k), d_delta_x(:,k), a_h, a, k_1, region);
dA=derivate_A(Delta_x(:,k), d_Delta_x(:,k), xdes(:,k), dxdes(:,k), a_r, a_t, region);
a11(k)=A(3,3);
da11(k)=dA(3,3);
ddxf(:,k)=derivate_dxf(w(k), dw(k), dxdes(:,k), ddxdes(:,k), A, dA);

end

figure(1)
plot(t,w,t,dw)
title('B:w R:dw');
xlabel('t')

figure(2)
plot(t,delta_x(1,:),t,d_delta_x(1,:))
title('B: deltax R: ddeltax');
xlabel('t')

figure(3)
plot(t,k_p,t,dk_p)
title('B: kp R: dkp');
xlabel('t')

figure(4)
plot(t,a11,t,da11)
title('B: A_(11) R: dA_(11)');
xlabel('t')

figure(5)
plot(t,dx_f(1,:),t,ddxf(1,:))
title('B: dxf R: ddxf');
xlabel('t')
