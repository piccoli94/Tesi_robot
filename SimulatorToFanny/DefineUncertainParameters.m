function UncertainDynamicParameters = DefineUncertainParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,NominalParam,radius_uncert,length_uncert,coglink_uncert,cogmotor_uncert,inermotor_uncert,fv_uncert,fc_uncert)

% UncertainDynamicParameters = DefineUncertainParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,NominalParam,radius_uncert,length_uncert,coglink_uncert,cogmotor_uncert,inermotor_uncert,fv_uncert,fc_uncert)
%
% Input parameters
%
% di: kinematic parameters
% ri: link radii
% h7: length of link 7 (flange)
% g: gravity acceleration
% mli: link masses
% NominalParam: nominal parameters: imi, fvi, fci
%
% radius_uncert [0,1]: uncertainty factor on radius dimension of cylindrical links (e.g. 0.01)
%
% length_uncert [0,1]: uncertainty factor on length of cylindrical links (e.g. 0.01)
%
% coglink_uncert [0,1]: uncertainty factor on COG of links (e.g. 0.02)
%
% cogmotor_uncert [0,1]: uncertainty factor on COG of motors (e.g. 0.01)
%
% inermotor_uncert [0,1]: uncertainty factor on moment of inertia of motors (e.g. 0.01)
%
% fv_uncert [0,1]: uncertainty factor on viscous friction coefficients (e.g. 0.02)
%
% fc_uncert [0,1]: uncertainty factor on Coulomb friction coefficients (e.g. 0.03)
%
% C. Natale, February 2015

% dimensions of the links 
% links radii (flange radius r7 to be checked)
r1=r1*(1+radius_uncert); r2=r2*(1-radius_uncert); r3=r3*(1+radius_uncert); r4=r4*(1-radius_uncert); r5=r5*(1+radius_uncert); r6=r6*(1-radius_uncert); r7=r7*(1+radius_uncert);
% link lengths
h1 = d3/2*(1+length_uncert); h2 = d3/2*(1-length_uncert); h3 = d3/2*(1+length_uncert); h4 = d5/2*(1-length_uncert); h5 = d5/2*(1+length_uncert); h7=h7*(1+length_uncert); 

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

% COG of links (expressed in link frame)
pl1y = -d0/3*(1+coglink_uncert); pl2z = d3/4*(1-coglink_uncert); pl3y = d3/4*(1+coglink_uncert); pl4z = d5/4*(1-coglink_uncert); pl5y = -d5/4*(1+coglink_uncert); pl7z = d7/8*(1-coglink_uncert);
% Motor positions (expressed in -preceeding- link frame) 
pm1z = -d0/3*(1+cogmotor_uncert); pm3z = d3/2*(1-cogmotor_uncert); pm5z = d5/2*(1+cogmotor_uncert);
% inertia of motor rotors  uncertainties of 15%
im1=NominalParam(32)*(1+inermotor_uncert); im2=NominalParam(33)*(1+inermotor_uncert); im3=NominalParam(34)*(1+inermotor_uncert); 
im4=NominalParam(35)*(1+inermotor_uncert); im5=NominalParam(36)*(1-inermotor_uncert); im6=NominalParam(37)*(1-inermotor_uncert); 
im7=NominalParam(38)*(1-inermotor_uncert); 

%Friction parameters taken from:
%A. Jubien, M. Gautier, A. Janot, Dynamic identification of the Kuka LWR robot using motor torques 
%and joint torque sensors data, Proc. of the 19th IFAC World Congress, Cape Town, 2014, pp. 8391-8396.
%Uncertainties from 20% to 30%
fv1 = NominalParam(39)*(1+fv_uncert); fv2 = NominalParam(40)*(1-fv_uncert); fv3 = NominalParam(41)*(1+fv_uncert); fv4 = NominalParam(42)*(1+fv_uncert); 
fv5 = NominalParam(43)*(1-fv_uncert); fv6 = NominalParam(44)*(1-fv_uncert); fv7 = NominalParam(45)*(1+fv_uncert);
fc1 = NominalParam(46)*(1+fc_uncert); fc2 = NominalParam(47)*(1-fc_uncert); fc3 = NominalParam(48)*(1+fc_uncert); fc4 = NominalParam(49)*(1-fc_uncert); 
fc5 = NominalParam(50)*(1+fc_uncert); fc6 = NominalParam(51)*(1-fc_uncert); fc7 = NominalParam(52)*(1+fc_uncert);

%Vector of dynamic parameters that are subject to uncertainty (all other
%parameters are fixed in the symbolic model)                      
UncertainDynamicParameters = [il1x il1y il1z il2x il2y il2z il3x il3y il3z il4x il4y il4z il5x il5y il5z il6x il6y il6z il7x il7y il7z...
                              g pl1y pl2z pl3y pl4z pl5y pl7z pm1z pm3z pm5z im1 im2 im3 im4 im5 im6 im7...
                              fv1 fv2 fv3 fv4 fv5 fv6 fv7 fc1 fc2 fc3 fc4 fc5 fc6 fc7];                       