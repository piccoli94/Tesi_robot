clear all
clc
close all
a_h=1;
a_r=5;
a_t=10;
k_1=2;
a=0.3;
Delta_x=ones(3,1).*(0:0.01:10);
xdes=ones(3,length(Delta_x));
dxdes=diff(xdes);
xe=xdes+Delta_x;
x_i=zeros(3,length(Delta_x));
for k=1:length(Delta_x)
region=dominant_region(Delta_x(:,k),a_h,a_r,a_t);
if (strcmp(region,'SS-SR'))
    x_i(:,k)=xe(:,k);
end
w(k)=weight_vector(Delta_x(:,k), region, a_r, a_t); 
norm_delta(k)=norm(Delta_x(:,k));
A(:,:,k)=transition_matrix_A(xe(:,k),xdes(:,k),w(k),a_r,a_t);
p(k)=A(1,1,k);
x_d(:,k)=x_i(:,k)+w(k)*(xdes(:,k)-x_i(:,k));
delta_x(:,k)=xe(:,k)-x_d(:,k);
k_p(k)=pos_stiffness(delta_x(:,k), k_1, a, a_h);
end

plot(norm_delta,p,'r')
hold on
plot(norm_delta(1:end-1),3*diff(w),'b')
hold off
legend('A11','diff(w)')

figure(2)
plot(x_d(1,:),'r')
hold on 
plot(x_i(1,:),'k')
plot(xdes(1,:),'g')
legend('xd','xi,','xdes')

figure(3)
plot(norm_delta,k_p)


q=zeros(7,1);
J = func_getJacobian(q,7)   
J=J(1:3,:)% Jacobian   (da testare)
A=A(:,:,1)
M=eye(7)-pinv(J)*A*J;                                     % Modifier Matrix (6)
dx_f=w*dxdes-A*dxdes;                                     % virtual position reference vector (8a)
dqr=M\(pinv(J)*dx_f-alpha*pinv(J)*k_p*delta_x);           % derivate of virtual joint reference vector (8b)
s=dq-dqr;      