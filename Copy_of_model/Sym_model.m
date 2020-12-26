clc
clear all
close all
N=7;
q=sym('q',[1,N],'real');
dq=sym('dq',[1,N],'real');
ddq=sym('ddq',[1,N],'real');

[T_ij,T_0j] = func_getTransformations_sym(q);
  T_ij=simplify(T_ij);
  T_0j=simplify(T_0j);
JointAxes = [0,0,1;
                0,1,0;
                0,0,1;
                0,-1,0;
                0,0,1;
                0,1,0;
                0,0,1];
noJoints = 7;

z = sym(zeros(3,1,noJoints));
p = sym(zeros(3,1,noJoints));
pci= sym('pci',[3,1,noJoints],'real'); pci(1:2,1,:)=0;
pci0= sym('pci0',[3,1,noJoints]);
kr=sym('kr',[1,1,noJoints],'real');
TCur = sym(eye(4));
for curJoint = 1:noJoints
        
        TCur = T_0j(:,:,curJoint);

        z(:,:,curJoint) = TCur(1:3,1:3)*JointAxes(curJoint,:)';
        
        % position of axis
        p(:,:,curJoint) = TCur(1:3,4);
        pci0(:,1,curJoint)=p(:,:,curJoint)+TCur(1:3,1:3)* pci(:,1,curJoint);
      
end
        Jpl = sym(zeros(3,noJoints,noJoints));
        Jpm = sym(zeros(3,noJoints,noJoints));
        Jol = sym(zeros(3,noJoints,noJoints));
        Jom = sym(zeros(3,noJoints,noJoints));
        
curJoint=1;
        Jpl(:,1,curJoint)=cross(z(:,:,curJoint), pci0(:,1,curJoint)-p(:,1,1));
        Jol(:,1,curJoint)=z(:,:,curJoint);
        Jom(:,1,curJoint)=kr(curJoint)*z(:,:,1);
        Jpm(:,1,curJoint)=zeros(3,1);
for curJoint=2:noJoints
    for k=curJoint:-1:1
    Jpl(:,k,curJoint)=cross(z(:,:,k), pci0(:,1,curJoint)-p(:,1,k));
    end
end
Jpl=simplify(Jpl);

for curJoint=2:noJoints
    for k=curJoint:-1:1
    Jol(:,k,curJoint)=z(:,:,k);
    end
end    

ml=sym('ml',[1,N],'real');
I=sym('Il',[1,3,N],'real');

for curJoint=1:noJoints
Il(:,:,curJoint)=diag([I(1,:,curJoint)]);
end
Blp=sym(zeros(7,7));
Blo=sym(zeros(7,7));
for curJoint=1:noJoints
    Blp=Blp+ml(curJoint)*Jpl(:,:,curJoint)'*Jpl(:,:,curJoint);
end

for curJoint=1:noJoints
    Blo=Blo+Jol(:,:,curJoint)'*T_0j(1:3,1:3,curJoint)'*Il(:,:,curJoint)*T_0j(1:3,1:3,curJoint)*Jol(:,:,curJoint);
end

for curJoint=2:noJoints
    for k=curJoint-1:-1:1
    Jpm(:,k,curJoint)=cross(z(:,:,k), p(:,1,curJoint)-p(:,1,k));
    end
end 
Jpm=simplify(Jpm);
for curJoint=2:noJoints
    for k=curJoint:-1:1
        if(k==curJoint)
            Jom(:,k,curJoint)=kr(curJoint)*z(:,:,curJoint); % [0;0;1] o uguale all'orientamento del giunto?
        else
            Jom(:,k,curJoint)=Jol(:,k,curJoint);
        end
    end
end 
Jom=simplify(Jom);

Bmp=sym(zeros(7,7));
Bmo=sym(zeros(7,7));

mm=sym('mm',[1,N],'real');
I2=sym('Im',[1,N],'real');
for curJoint=1:noJoints
Im(:,:,curJoint)=diag([0,0,I2(curJoint)]);
end


for curJoint=1:noJoints
    Bmp=Bmp+mm(curJoint)*Jpm(:,:,curJoint)'*Jpm(:,:,curJoint);
end
for curJoint=1:noJoints
    Bmo=Bmo+Jom(:,:,curJoint)'*T_0j(1:3,1:3,curJoint)'*Im(:,:,curJoint)*T_0j(1:3,1:3,curJoint)*Jom(:,:,curJoint);
end

%---------------------------------------------------------------------
%gravity
g=sym(zeros(7,1));
g0=sym('g0','real');
gv=[0,0,-g0];
for j=1:7
    for curJoint=1:noJoints
    g(j)=g(j)+ml(curJoint)*gv*Jpl(:,j,curJoint)+mm(curJoint)*gv*Jpm(:,j,curJoint);
    end
end
g=-g;
%-------------------------------------------------------------
% Blp=simplify(Blp);
% Blo=simplify(Blo);
% Bmp=simplify(Bmp);
% Bmo=simplify(Bmo);
g=simplify(g);
%--------------------------------------------------------------
hlp=sym(zeros(7,7));
hlo=sym(zeros(7,7));
hmp=sym(zeros(7,7));
hmo=sym(zeros(7,7));

for i=1:7
    for j=1:7
        for k=1:7
            hlp(i,j)=hlp(i,j)+(diff(Blp(i,j),q(k))-0.5*diff(Blp(j,k),q(i)))*dq(k);
            hlo(i,j)=hlo(i,j)+(diff(Blo(i,j),q(k))-0.5*diff(Blo(j,k),q(i)))*dq(k);
            hmp(i,j)=hmp(i,j)+(diff(Bmp(i,j),q(k))-0.5*diff(Bmp(j,k),q(i)))*dq(k);
            hmo(i,j)=hmo(i,j)+(diff(Bmo(i,j),q(k))-0.5*diff(Bmo(j,k),q(i)))*dq(k);
        end
    end
end
B=(Blp+Blo+Bmp+Bmo);
C=(hlp+hlo+hmp+hmo);

dqr=sym('dqr',[1,N],'real');
ddqr=sym('ddqr',[1,N],'real');

tau=B*ddqr'+C*dqr'+g;
%tau=simplify(tau);

save('B', 'Blp','Blo','Bmp','Bmo','B');
save('C', 'hlp','hlo','hmp','hmo','C');
save('G','g');
save('tau','tau');
