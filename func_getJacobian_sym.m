%% func_getJacobian.m
% this function calculates the Jacobian using MATLAB-Code

function J = func_getJacobian_sym(q)

   [T_ij,T_0j] = func_getTransformations_sym(q);
   
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
    v = sym(zeros(3,1,noJoints));
    p = sym(zeros(3,1,noJoints));
    
    TCur = sym(eye(4));
        
    for curJoint = 1:noJoints
        
        TCur = TCur*T_ij(:,:,curJoint);

        v(:,:,curJoint) = TCur(1:3,1:3)*JointAxes(curJoint,:)';
        
        % position of axis
        p(:,:,curJoint) = TCur(1:3,4);
      
    end
    
    % apply the final transformation
    %TCur = TCur*T_linkSystem_desiredFrame; BISOGNA TESTARE QUESTA PARTE SE
    %SERVE O NO
    
    T_0EE = TCur*T_ij(:,:,8);
    s=T_0EE(1:3,4);
    J = sym(zeros(6,noJoints));
        for curJoint = 1:noJoints
            % it's a revolute joint
                J(1:3,curJoint) = cross(v(:,:,curJoint),s-p(:,:,curJoint));
                J(4:6,curJoint) = v(:,:,curJoint);
        end
       
    
    
    % Jacobian numerically;
    
%     if bruteSolution
%         JStar = zeros(6,noJoints);
%         
%         tic
%         for curJoint = 1:noJoints
%             delta_q = 0.0001;
%             oneVec = zeros(size(q));
%             oneVec(curJoint) = 1;
%             qStar = q + delta_q*oneVec;
%             TStar_ij = func_getTransformations(robot,qStar);
%             
%             TCurStar = eye(4);
%             for curJointStar = 1:bodyID
%                 TCurStar = TCurStar*TStar_ij(:,:,curJointStar);
%                 %             vStar(:,:,curJointStar) = TCurStar(1:3,3);  % z-axis is joint axis
%                 %             pStar(:,:,curJointStar) = TCurStar(1:3,4);  % position of axis
%             end
%             TCurStar = TCurStar*T_linkSystem_desiredFrame;
%             
%             % interesting point:
%             sStar = TCurStar(1:3,4);
%             
%             JStar(1:3,curJoint) = (sStar-s)/delta_q;
%             
%             dR = TCurStar(1:3,1:3) - TCur(1:3,1:3);
%             omega_skew = (dR/delta_q)*TCur(1:3,1:3)';
%             JStar(4:6,curJoint) = omega_skew([6,7,2]);
%             
%         end
%         
%         tBrute = toc;
%         disp(['Timing for brute solution: ',num2str(tBrute)]);
%         disp(JStar);
%     end
    
    


