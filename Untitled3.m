clear all
clc
close all
a_h=1;
a_r=5;
a_t=10;
k_1=5;
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
x_d(:,k)=x_i(:,k)+w(k)*(xdes(:,k)-x_i(:,k));
delta_x(:,k)=xe(:,k)-x_d(:,k);
k_p(k)=pos_stiffness(delta_x(:,k), k_1, a, a_h);
dk_p(k)=derivate_kp(delta_x(:,k), a_h, a, k_1);

end

figure(3)
plot(norm_delta,k_p)
hold on
plot(norm_delta,dk_p)

