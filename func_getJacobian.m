%% func_getJacobian.m
% this function calculates the Jacobian using MATLAB-Code

function J = func_getJacobian(robot,q,bodyID,T_linkSystem_desiredFrame)

    screwSolution = 1;
    bruteSolution = 0;
    
    tic
    T_ij = robot.T;
    
    noJoints = robot.NoDofs;
    
    % Joint axes: position and direction vectors
    v = NaN(3,1,noJoints);
    p = NaN(3,1,noJoints);
    
    TCur = eye(4);
        
    for curJoint = 1:bodyID
        
        TCur = TCur*T_ij(:,:,curJoint);
        
        % the direction of the joint axis w.r.t the global system
        % both for a revolute and a prismatic joint
        v(:,:,curJoint) = TCur(1:3,1:3)*robot.JointAxes(curJoint,:)';
        
        % position of axis
        p(:,:,curJoint) = TCur(1:3,4);
      
    end
    
    % apply the final transformation
    TCur = TCur*T_linkSystem_desiredFrame;
    
    s = TCur(1:3,4);
    
    if screwSolution
        J = zeros(6,noJoints);
        for curJoint = 1:bodyID
            if robot.JointTypes(curJoint) == 1
                % it's a revolute joint
                J(1:3,curJoint) = cross(v(:,:,curJoint),s-p(:,:,curJoint));
                J(4:6,curJoint) = v(:,:,curJoint);
            else
                % it's a prismatic joint
                J(1:3,curJoint) = v(:,:,curJoint);
                J(4:6,curJoint) = zeros(3,1);
            end
            
        end
        
        tScrew = toc;
             disp(['Timing for screw solution: ',num2str(tScrew)]);
        %     disp(['Rank is ',num2str(rank(J))]);
    end
    
    
    % Jacobian numerically;
    
    if bruteSolution
        JStar = zeros(6,noJoints);
        
        tic
        for curJoint = 1:noJoints
            delta_q = 0.0001;
            oneVec = zeros(size(q));
            oneVec(curJoint) = 1;
            qStar = q + delta_q*oneVec;
            TStar_ij = func_getTransformations(robot,qStar);
            
            TCurStar = eye(4);
            for curJointStar = 1:bodyID
                TCurStar = TCurStar*TStar_ij(:,:,curJointStar);
                %             vStar(:,:,curJointStar) = TCurStar(1:3,3);  % z-axis is joint axis
                %             pStar(:,:,curJointStar) = TCurStar(1:3,4);  % position of axis
            end
            TCurStar = TCurStar*T_linkSystem_desiredFrame;
            
            % interesting point:
            sStar = TCurStar(1:3,4);
            
            JStar(1:3,curJoint) = (sStar-s)/delta_q;
            
            dR = TCurStar(1:3,1:3) - TCur(1:3,1:3);
            omega_skew = (dR/delta_q)*TCur(1:3,1:3)';
            JStar(4:6,curJoint) = omega_skew([6,7,2]);
            
        end
        
        tBrute = toc;
        disp(['Timing for brute solution: ',num2str(tBrute)]);
        disp(JStar);
    end
    
    


