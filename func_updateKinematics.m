%% func_updateKinematics.m
% depending on _calcFlag_ the function updates the commited robot-model,
% considering the new joint-coordinates _q_,_qDot_ and _qDotDot_

function robot = func_updateKinematics(robot,q,qDot,qDotDot)

    [robot.T,robot.T_0j] = func_getTransformations(robot, q);
    
