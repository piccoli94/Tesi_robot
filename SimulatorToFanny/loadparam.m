clear all,close all,clc

% kinematic parameters. WARNING: do not change since they are hard-coded in the symbolic model!
d0 = 0.3105;
d3 = 0.400;
d5 = 0.390;
d7 = 0.078;

%Initial robot configuration
q0 = [0 pi/2-0.3 0 0.3 0 0 0]';
%q0 = [pi/7 -pi/3 pi/2 0 0 0 0]';
%q0 = [pi/7 -pi/4 -pi/7 0 0 0 0]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% These lines require Robotics Toolbox but can be commented
% DH table:  q d  a  alpha
L(1) = Link([0 0  0  pi/2]);
L(2) = Link([0 0  0 -pi/2]);
L(3) = Link([0 d3 0 -pi/2]);
L(4) = Link([0 0  0  pi/2]);
L(5) = Link([0 d5 0  pi/2]);
L(6) = Link([0 0  0 -pi/2]);
L(7) = Link([0 0  0    0 ]);
LWR = SerialLink(L,'name','LWR4+');
LWR.base = transl(0, 0, d0);
LWR.tool = transl(0, 0, d7);
%LWR.plot(q0')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization of dynamic parameters
g = 9.81;
% masses of links (as provided by Fanny)
% WARNING: used only to compute intertia moments of links and hard coded in the rest of the symbolic model
ml1=2; ml2=2; ml3=2; ml4=2; ml5=2; ml6=0.02;ml7=0.1; 
% masses of motors (as provided by Fanny)
% WARNING: not used since they are hard-coded in the symbolic model
mm1=1; mm2=1; mm3=1; mm4=1; mm5=1; mm6=0.7; mm7=0.5; 
% link radii (flange radius r7 to be checked)
r1=0.05; r2=0.05; r3=0.05; r4=0.05; r5=0.05; r6=0.05; r7=0.02;  % links radius (flange radius r7 to be checked)
% link 7 length (to be checked). Other link lengths are based on kinematic parameters
h7=0.02; 

% inertia of motor rotors (as provided by Fanny)
im1=0.0001; im2=0.0001; im3=0.0001; im4=0.0001; im5=0.0001; im6=0.0001; im7=0.0001; 

%Friction parameters taken from:
%A. Jubien, M. Gautier, A. Janot, Dynamic identification of the Kuka LWR robot using motor torques 
%and joint torque sensors data, Proc. of the 19th IFAC World Congress, Cape Town, 2014, pp. 8391-8396.
fv1 = 14.4; fv2 = 15.3; fv3 = 6.55; fv4 = 11.0; fv5 = 4.29; fv6 = 2.26; fv7 =1.6;
fc1 = 11.9; fc2 = 11.5; fc3 = 8.98; fc4 = 8.35; fc5 = 8.31; fc6 = 4.72; fc7 = 6.04;

%Definition of the nominal parameter vector: these parameters are those used in the controller
NominalDynamicParameters = DefineNominalParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,...
                                    im1,im2,im3,im4,im5,im6,im7,fv1,fv2,fv3,fv4,fv5,fv6,fv7,fc1,fc2,fc3,fc4,fc5,fc6,fc7);
%Definition of the uncertain parameter vector: these parameters are those used in the robot dynamic model
UncertainDynamicParameters = DefineUncertainParameters(d0,d3,d5,d7,r1,r2,r3,r4,r5,r6,r7,h7,g,ml1,ml2,ml3,ml4,ml5,ml6,ml7,...
                                                       NominalDynamicParameters,0.0,0.0,0.1,0.1,0.2,0.3,0.3);

%The last 7 inputs are the uncertainty factors (see function help)



%For a perfect compensation uncomment the following line
UncertainDynamicParameters = NominalDynamicParameters;

%Friction flag: 1 to activate dry friction in the dynamic model (very slow simulation!)
ACTIVATE_COULOMB_FRICTION = 0; 

% Reduction ratios (as provided by Fanny)
% WARNING: do not change since they are hard-coded in the symbolic model
km1=150; km2=150; km3=150; km4=150; km5=150; km6=150; km7=150;
Km = diag([km1 km2 km3 km4 km5 km6 km7]);
% Parameters for the elastic joint model 
% Joint stiffness (taken from A. Jubien, M. Gautier, A. Janot, "Dynamic identification of the Kuka LWR robot
% using motor torques and joint torque sensors data", 19th IFAC WC, pp.8391-8396, Cape Town, 2014.)
Kj = diag([1e4 1e4 1e4 1e4 1e4 7.5e3 7.5e3]); 
im1 = UncertainDynamicParameters(32);
im2 = UncertainDynamicParameters(33);
im3 = UncertainDynamicParameters(34);
im4 = UncertainDynamicParameters(35);
im5 = UncertainDynamicParameters(36);
im6 = UncertainDynamicParameters(37);
im7 = UncertainDynamicParameters(38);
%motor inertia at the joint side
B =  Km*diag([im1 im2 im3 im4 im5 im6 im7])*Km; 

%Gearbox torque loss model (note that the model is written after the
%gearbox, thus it is written in the function losses as it was Km = 1.
eta = 0.95;
omegaT = 1e-3;

%Sampling time for control
Ts = 2e-3;

%Residual gain
Ki = 15*eye(7);
