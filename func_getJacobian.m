%% func_getJacobian.m
% depending on the _calcFlag_ this function either calculates the Jacobian
% using MATLAB-Code or calls RBDL-wrapper-function getJacobian.
%
% WARNING: RBDL-wrapper-function getJacobian only uses the translatory
% coordinates of _T_linkSystem_desiredFrame_, ignoring the rotational part!
function J = func_getJacobian(q)

   [T_ij,T_0j] = func_getTransformations(q);
   
JointAxes = [0,0,1;
                0,1,0;
                0,0,1;
                0,-1,0;
                0,0,1;
                0,1,0;
                0,0,1];
            
    screwSolution = 1;
   
    noJoints = 7;
    
    % Joint axes: position and direction vectors
    v = zeros(3,1,noJoints);
    p = zeros(3,1,noJoints);
    
    TCur = eye(4);
        
    for curJoint = 1:noJoints
        
        TCur = TCur*T_ij(:,:,curJoint);

        v(:,:,curJoint) = TCur(1:3,1:3)*JointAxes(curJoint,:)';
        
        % position of axis
        p(:,:,curJoint) = TCur(1:3,4);
      
    end
    
    % apply the final transformation
    %TCur = TCur*T_linkSystem_desiredFrame;
    
    T_0EE = TCur*T_ij(:,:,8);
    s=T_0EE(1:3,4);
    J = zeros(6,noJoints);
        for curJoint = 1:noJoints
            % it's a revolute joint
                J(1:3,curJoint) = cross(v(:,:,curJoint),s-p(:,:,curJoint));
                J(4:6,curJoint) = v(:,:,curJoint);
                              
               
                
        end
        