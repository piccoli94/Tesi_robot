
q=ones(7,1)*pi/2;
dq=ones(7,1)*1/2;
dqr=ones(7,1)*1/2.1;
pci=ones(3,7)*0.5;
ddqr=ones(7,1)*1/10;

y1=Y_b1(q,dq,dqr,ddqr,pci);
y2=Y_b2(q,dq,dqr,ddqr,pci);
y3=Y_b3(q,dq,dqr,ddqr,pci);