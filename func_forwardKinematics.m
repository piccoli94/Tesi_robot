% the Kinematics are calculated in MATLAB
% bodyID- which manipulator part you considered
function [x,R] = func_forwardKinematics(robot,q,bodyID,T_body_desiredFrame)

    noJoints = robot.NoDofs;

    if (1 <= bodyID) && (bodyID <= noJoints)
        
        TCur = robot.T_0j(:,:,bodyID)*T_body_desiredFrame;

        R = TCur(1:3,1:3);
        x = TCur(1:3,4);

    elseif bodyID == 0
       msg = 'bodyID > 0 must be a positive value';
        error(msg);
    else
        msg = 'bodyID must be an positive integer';
        error(msg);
    end
       