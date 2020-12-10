% the Kinematics are calculated in MATLAB
% bodyID- which manipulator part you considered
function [x,R] =func_forwardKinematics(q)

    [T_ij,T_0j] = func_getTransformations(q);
        
        TCur = T_0j(:,:,8);

        R = TCur(1:3,1:3);
        x = TCur(1:3,4);

   
       