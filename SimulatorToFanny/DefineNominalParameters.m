function NominalDynamicParameters = DefineNominalParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,im1,im2,im3,im4,im5,im6,im7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,fc1,fc2,fc3,fc4,fc5,fc6,fc7)

% NominalDynamicParameters = DefineNominalParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,im1,im2,im3,im4,im5,im6,im7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,fc1,fc2,fc3,fc4,fc5,fc6,fc7)
%
% Input parameters
%
% di: kinematic parameters
% ri: link radii
% h7: length of link 7 (flange)
% g: gravity acceleration
% mli: link masses
% imi: inertia moments of motors
% fvi: viscous friction coefficients
% fci: Coulomb friction coefficients

% Dynamic Parameters of the Kuka LWR4+ (nominal numerical values)
% Assumptions:
%   - cylindrical links from 1 to 5 and 7 (only flange)
%   - spherical link 6
%   - motor of joint i is placed on link i-1
%   - COG positions of links and motors are very rough 
%   - link dimensions are very rough
% DH table:    q d  a  alpha
% L(1) = Link([0 0  0  pi/2]);
% L(2) = Link([0 0  0 -pi/2]);
% L(3) = Link([0 d3 0 -pi/2]);
% L(4) = Link([0 0  0  pi/2]);
% L(5) = Link([0 d5 0  pi/2]);
% L(6) = Link([0 0  0 -pi/2]);
% L(7) = Link([0 0  0     0]);
%  d0 and d7 are in the Tb0 and Te7 HT
%
% C. Natale, February 2015

 % dimensions of the links
 h1 = d3/2; h2 = d3/2; h3 = d3/2; h4 = d5/2; h5 = d5/2; % link lengths 
 
 %inertia of the "cylinder" links
 il1x=1/12*ml1*(3*r1^2+h1^2);
 il1y=1/2*ml1*r1^2;
 il1z=1/12*ml1*(3*r1^2+h1^2);
 il2x=1/12*ml2*(3*r2^2+h2^2);
 il2y=1/12*ml2*(3*r2^2+h2^2);
 il2z=1/2*ml2*r2^2;
 il3x=1/12*ml3*(3*r3^2+h3^2);
 il3y=1/2*ml3*r3^2;
 il3z=1/12*ml3*(3*r3^2+h3^2);
 il4x=1/12*ml4*(3*r4^2+h4^2);
 il4y=1/12*ml4*(3*r4^2+h4^2);
 il4z=1/2*ml4*r4^2;
 il5x=1/12*ml5*(3*r5^2+h5^2);
 il5y=1/2*ml5*r5^2; 
 il5z=1/12*ml5*(3*r5^2+h5^2);
 %inertia of the sixth "sphere" link
 il6x=2/5*ml6*r6^2; 
 il6y=2/5*ml6*r6^2;
 il6z=2/5*ml6*r6^2;
 %inertia of the seventh link (flange)
 il7x=1/12*ml7*(3*r7^2+h7^2);
 il7y=1/12*ml5*(3*r7^2+h7^2);
 il7z=1/2*ml7*r7^2;
 
 % COG of links (expressed in link frame) zeros are hard-coded in the symbolic model!
 pl1x = 0; pl1y = -d0/3; pl1z =0; pl2x = 0; pl2y = 0; pl2z = d3/4; pl3x = 0; pl3y = d3/4; pl3z = 0; pl4x = 0; pl4y = 0; pl4z = d5/4; 
 pl5x = 0; pl5y = -d5/4; pl5z = 0; pl6x = 0; pl6y =0; pl6z = 0; pl7x = 0; pl7y = 0; pl7z = d7/8;
 % Motor positions (expressed in -preceeding- link frame) zeros are hard-coded in the symbolic model!
 pm1x = 0; pm1y = 0; pm1z = -d0/3; pm2x = 0; pm2y = 0; pm2z = 0; pm3x = 0; pm3y = 0; pm3z = d3/2; pm4x = 0; pm4y = 0; pm4z = 0; 
 pm5x = 0; pm5y = 0; pm5z = d5/2; pm6x = 0; pm6y = 0; pm6z = 0; pm7x = 0; pm7y = 0; pm7z = 0;

%Vector of dynamic parameters that can be subject to uncertainty (all other
%parameters are hard-coded in the symbolic model)
NominalDynamicParameters = [il1x il1y il1z il2x il2y il2z il3x il3y il3z il4x il4y il4z il5x il5y il5z il6x il6y il6z il7x il7y il7z...
                            g pl1y pl2z pl3y pl4z pl5y pl7z pm1z pm3z pm5z im1 im2 im3 im4 im5 im6 im7...
                            fv1 fv2 fv3 fv4 fv5 fv6 fv7 fc1 fc2 fc3 fc4 fc5 fc6 fc7];                       