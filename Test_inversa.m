close all
q0=[0,pi/2,0,-pi/4,0,-pi/2,0];
xm=zeros(3,length(xe)+1);
xm(:,1)=func_forwardKinematics(q0);
K=100*diag(ones(1,3));
j=func_getJacobian(q0);
j=j(1:3,:);
jplus=J_pseudoinverse(j);
q=zeros(7,length(xe)+1);
q(:,1)=q0';
e=zeros(3,length(xe)+1);

d(1)=det(j*j');

% qs=sym('q',[1,7]);
% fun = @(q) -det(func_getJacobian_sym(qs)*func_getJacobian_sym(qs)');
% lb=[-pi,-pi,-pi,-pi,-pi,-pi,-pi];
% ub=[pi,pi,pi,pi,pi,pi,pi];
% 
% Aeq = [];
% beq = [];
% 
% A = [];
% b= [];
% 
% [qstar,f,ef,out,l,G,H] = -fmincon(fun,q(:,1),A,b,Aeq,beq,lb,ub);
for k=1:length(xe)
 e(:,k)=xe(:,k)-xm(:,k);
dq(:,k)=jplus*(K*e(:,k)+dxe(:,k)); %+ (eye(7)-jplus*j)*dqstar;
q(:,k+1)=q(:,k)+ dq(:,k)*0.001;
%j0 = R.jacob0(q0);
j=func_getJacobian(q(:,k+1));
j=j(1:3,:);
d(k)=det(j*j');
jplus=J_pseudoinverse(j);
xm(:,k+1)=func_forwardKinematics(q(:,k+1));
end
plot3(xm(1,:),xm(2,:),xm(3,:))
hold on
plot3(xe(1,:),xe(2,:),xe(3,:))
xlabel('x') 
ylabel('y')
figure(2)
plot(d)

ddq=[diff(dq')',zeros(7,1)]/0.001;
dxm=[diff(xm')',zeros(3,1)]/0.001;
ddxm=[diff(dxm')',zeros(3,1)]/0.001;

xm=xm(:,500:end-10);
dxm=dxm(:,500:end-10);
ddxm=ddxm(:,500:end-10);
q=q(:,500:end-10);
dq=dq(:,500:end-9);
ddq=ddq(:,500:end-9);

save('Ref','xm','dxm','ddxm','q','dq','ddq')