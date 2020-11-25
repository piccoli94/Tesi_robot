% the Kinematics are calculated in MATLAB
% bodyID- which manipulator part you considered
function [x,R] =func_forwardKinematics(q,bodyID)

    [T_ij,T_0j] = func_getTransformations(q);
    noJoints = 7;

    if (1 <= bodyID) && (bodyID <= noJoints)
        
        TCur = T_0j(:,:,bodyID);

        R = TCur(1:3,1:3);
        x = TCur(1:3,4);

    elseif bodyID == 0
       msg = 'bodyID > 0 must be a positive value';
        error(msg);
    else
        msg = 'bodyID must be an integer';
        error(msg);
    end
       