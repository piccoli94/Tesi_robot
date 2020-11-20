%% func_getTransformations.m
% The function calculates the transformationmatrices _T_ij_ of the _robot_
% according to the given joint-configuration _q_

function [T_ij,T_0j] = func_getTransformations(robot,q)

noJoints = robot.NoDofs;

T_ij = NaN(4,4,noJoints);

for curJoint = 1:noJoints
    
    if robot.JointTypes(curJoint) == 1
        % it's a revolute joint
        angles = robot.JointAxes(curJoint,:)*q(curJoint);
        distance = robot.Lengths(curJoint,:);
        T_ij(:,:,curJoint) = func_make_translation(distance)*func_make_rotation('xyz', angles);
    else
        % it's a prismatic joint
        distance = robot.Lengths(curJoint,:) + robot.JointAxes(curJoint,:)*q(curJoint);
        T_ij(:,:,curJoint) = func_make_translation(distance);
    end    
    
end

T_End_Flange = robot.T_finalLink_Flange;

T_0j = NaN(4,4,noJoints+1);
T_0j(:,:,1) = T_ij(:,:,1);
    
for i = 2:noJoints
    T_0j(:,:,i) = T_0j(:,:,i-1)*T_ij(:,:,i);
end

T_0j(:,:,end) = T_0j(:,:,end-1)*T_End_Flange;