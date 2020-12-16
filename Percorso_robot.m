clc
clear all
P0=[200e-3;
    200e-3;
    300e-3];
Pm=[-200e-3;
    -200e-3;
     650e-3];
Pf=[200e-3;
    200e-3;
   1000e-3];
 s0=0; sf1=norm(P0-Pm); sf2=norm(Pm-Pf);
[s1,sd1,sdd1] = lspb(s0,sf1,10);
[s2,sd2,sdd2] = lspb(s0,sf2,10);

P1x=P0(1)*cos(pi*(s1/sf1));
P1y=P0(2)*sin(pi*(s1/sf1));
P1z=P0(3)+s1*(Pm(3)-P0(3))/sf1;

dP1x=-P0(1)*sd1.*sin(pi*(s1./sf1))*pi/sf1;
dP1y= P0(2)*sd1.*cos(pi*(s1./sf1))*pi/sf1;
dP1z=sd1*(Pm(3)-P0(3))/sf1;

ddP1x=-P0(1)*sd1.^2.*cos(pi*(s1./sf1))*(pi/sf1)^2 -P0(1)*sdd1.*sin(pi*(s1./sf1))*pi/sf1;
ddP1y=-P0(2)*sd1.^2.*sin(pi*(s1./sf1))*(pi/sf1)^2 +P0(2)*sdd1.*cos(pi*(s1./sf1))*pi/sf1;
ddP1z=sdd1*(Pm(3)-P0(3))/sf1;

P2x=Pm(1)*cos(pi*(s2/sf2));
P2y=Pm(2)*sin(pi*(s2/sf2));
P2z=Pm(3)+s2*(Pf(3)-Pm(3))/sf2;

dP2x=-Pm(1)*sd2.*sin(pi*(s2/sf2))*pi/sf2;
dP2y= Pm(2)*sd2.*cos(pi*(s2/sf2))*pi/sf2;
dP2z=sd2*(Pf(3)-Pm(3))/sf2;

ddP2x=-P0(1)*sd2.^2.*cos(pi*(s2/sf2))*(pi/sf2)^2 -Pm(1)*sdd2.*sin(pi*(s2/sf2))*pi/sf2;
ddP2y=-P0(2)*sd2.^2.*sin(pi*(s2/sf2))*(pi/sf2)^2 +Pm(2)*sdd2.*cos(pi*(s2/sf2))*pi/sf2;
ddP2z=sdd2*(Pf(3)-Pm(3))/sf2;

Ps=[P1x',P2x(2:end)';
    P1y',P2y(2:end)';
    P1z',P2z(2:end)'];

dPs=[dP1x',dP2x(2:end)';
     dP1y',dP2y(2:end)';
     dP1z',dP2z(2:end)'];
 
ddPs=[ddP1x',ddP2x(2:end)';
      ddP1y',ddP2y(2:end)';
      ddP1z',ddP2z(2:end)'];

plot3(Ps(1,:),Ps(2,:),Ps(3,:))
hold on
plot3(dPs(1,:),dPs(2,:),dPs(3,:))
plot3(ddPs(1,:),ddPs(2,:),ddPs(3,:))
hold off


x =linspace(0,5,19);
ts=0:0.001:5;
Psx_int = interpn(x,Ps(1,:),ts,'spline');
Psy_int = interpn(x,Ps(2,:),ts,'spline');
Psz_int = interpn(x,Ps(3,:),ts,'spline');


dPsx_int = interpn(x,dPs(1,:),ts,'spline');
dPsy_int = interpn(x,dPs(2,:),ts,'spline');
dPsz_int = interpn(x,dPs(3,:),ts,'spline');

ddPsx_int = interpn(x,ddPs(1,:),ts,'spline');
ddPsy_int = interpn(x,ddPs(2,:),ts,'spline');
ddPsz_int = interpn(x,ddPs(3,:),ts,'spline');

figure(2)
plot3(Psx_int,Psy_int,Psz_int)
figure(3)
plot3(dPsx_int,dPsy_int,dPsz_int)
figure(4)
plot3(ddPsx_int,ddPsy_int,ddPsz_int)

xe=[Psx_int;Psy_int;Psz_int];
dxe=[dPsx_int;dPsy_int;dPsz_int];
ddxe=[ddPsx_int;ddPsy_int;ddPsz_int];