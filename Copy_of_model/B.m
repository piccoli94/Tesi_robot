N=7;
q=sym('q',[1,N]);
dq=sym('dq',[1,N]);
ddq=sym('ddq',[1,N]);
ml=sym('Ml',[1,N]);
mm=sym('Mm',[1,N]);
Il=sym('Il',[3,3,N]);
Im=sym('Im',[3,3,N]);
fm=sym('Fm',[1,N]);
[T_ij,T_0j] = func_getTransformations_sym(q);
   
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
pci= sym('pci',[3,1,noJoints]);
pci0= sym('pci0',[3,1,noJoints]);
pm0= sym('pm0',[3,1,noJoints]);
kr= sym('kr',[1,1,noJoints]);

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
for curJoint=2:noJoints
    for k=curJoint:-1:1
    Jpl(:,k,curJoint)=cross(z(:,:,k), pci0(:,1,curJoint)-p(:,1,k));
    end
end
for curJoint=2:noJoints
    for k=curJoint:-1:1
    Jol(:,k,curJoint)=z(:,:,k);
    end
end    
for curJoint=2:noJoints
    for k=curJoint-1:-1:1
    Jpm(:,k,curJoint)=cross(z(:,:,k), pm0(:,1,curJoint)-p(:,1,k));
    end
end 
for curJoint=2:noJoints
    for k=curJoint:-1:1
        if(k==curJoint)
            Jom(:,k,curJoint)=kr(curJoint)*z(:,:,1); 
        else
            Jom(:,k,curJoint)=Jol(:,k,curJoint);
        end
    end
end 
B=sym(zeros(7,7));
for curJoint=1:noJoints
    B=B+ml(curJoint)*Jpl(:,:,curJoint)'*Jpl(:,:,curJoint) + Jol(:,:,curJoint)'*Il(:,:,curJoint)*Jol(:,:,curJoint) +...
        mm(curJoint)*Jpm(:,:,curJoint)'*Jpm(:,:,curJoint) + Jom(:,:,curJoint)'*Im(:,:,curJoint)*Jom(:,:,curJoint);
end