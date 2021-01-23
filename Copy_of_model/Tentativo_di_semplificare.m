clc
clear all
close all
N=7;
q=sym('q',[1,N],'real');
dq=sym('dq',[1,N],'real');
ddq=sym('ddq',[1,N],'real');

d0=0.34; 
de=0.035+0.126;
d    =[    0,    0,  0.4,     0,   0.4,    0,     0];
a    =[    0,    0,    0,     0,     0,    0,     0];
theta=[ q(1), q(2), q(3),  q(4),  q(5), q(6),  q(7)];
alpha=[-pi/2, pi/2, pi/2, -pi/2, -pi/2, pi/2,     0];

Tij=sym(zeros(4,4,7));
T0j=sym(zeros(4,4,7));

for j=1:N
Tij(:,:,j)=[cos(theta(j)), -sin(theta(j))*cos(alpha(j)),  sin(theta(j))*sin(alpha(j)), a(j)*cos(theta(j));
            sin(theta(j)),  cos(theta(j))*cos(alpha(j)), -cos(theta(j))*sin(alpha(j)), a(j)*sin(theta(j));
                        0,                sin(alpha(j)),                cos(alpha(j)),               d(j);
                        0,                            0,                            0,                 1];
if(alpha(j)==pi/2 || alpha(j)==-pi/2)
    Tij(1:2,2,j)=0; Tij(3,3,j)=0;
end
end
Tij=simplify(Tij);

Tb0=sym(eye(4)); Tb0(3,4)=d0;
Tje=sym(eye(4)); Tje(3,4)=de;

T0j(:,:,1)=Tij(:,:,1);
for j=2:N
    T0j(:,:,j)=T0j(:,:,j-1)*Tij(:,:,j);
end
T0j=simplify(T0j);


Tbe=Tb0*T0j(:,:,end)*Tje;
T0e=T0j(:,:,end)*Tje;
p0e=T0e(1:3,4);
% temp(q)=Tbe;
% double(temp(pi/2,pi/2,pi/2,pi/2,pi/2,pi/2,pi/2))
z=sym(zeros(3,N));
z(:,1)=[0;0;1];
z(:,2:7)=T0j(1:3,3,1:6);
z=simplify(z);

p=sym(zeros(3,N));
p(:,1)=zeros(3,1);
p(:,2:8)=T0j(1:3,4,1:7);

J=sym(zeros(6,7));

for j=1:N
    J(1:3,j)=cross(z(:,j), p0e-p(:,j));
    J(4:6,j)=z(:,j);
end

 % COG of links (expressed in link frame) zeros are hard-coded in the symbolic model!
pl1x = 0; pl1y = -d0/3; pl1z =0; pl2x = 0; pl2y = 0; pl2z = d(3)/4; pl3x = 0; pl3y = d3/4; pl3z = 0; pl4x = 0; pl4y = 0; pl4z = d(5)/4; 
 pl5x = 0; pl5y = -d(5)/4; pl5z = 0; pl6x = 0; pl6y =0; pl6z = 0; pl7x = 0; pl7y = 0; pl7z = d(7)/8;
 
 pl1=[pl1x;pl1y;pl1z]; pl2=[pl2x;pl2y;pl2z]; pl3=[pl3x;pl3y;pl3z]; pl4=[pl4x;pl4y;pl4z]; pl5=[pl5x;pl5y;pl5z]; pl6=[pl6x;pl6y;pl6z]; pl7=[pl7x;pl7y;pl7z];
 % Motor positions (expressed in -preceeding- link frame) zeros are hard-coded in the symbolic model!
 pm1x = 0; pm1y = 0; pm1z = -d0/3; pm2x = 0; pm2y = 0; pm2z = 0; pm3x = 0; pm3y = 0; pm3z = d(3)/2; pm4x = 0; pm4y = 0; pm4z = 0; 
 pm5x = 0; pm5y = 0; pm5z = d(5)/2; pm6x = 0; pm6y = 0; pm6z = 0; pm7x = 0; pm7y = 0; pm7z = 0;
 
 pm1=[pm1x;pm1y;pm1z]; pm2=[pm2x;pm2y;pm2z]; pm3=[pm3x;pm3y;pm3z]; pm4=[pm4x;pm4y;pm4z]; pm5=[pm5x;pm5y;pm5z]; pm6=[pm6x;pm6y;pm6z]; pm7=[pm7x;pm7y;pm7z];

 