%% func_getTransformations.m
% The function calculates the transformationmatrices _T_ij_ of the _robot_
% according to the given joint-configuration _q_

function [T_ij,T_0j] = func_getTransformations_sym(q)

noJoints = 7;

Lengths = ...   
               [0.0000e-3,   0.0000e-3, 0e-3;
                0.0000e-3,   0.0000e-3, 0e-3;
                0.0000e-3,   0.0000e-3, 400e-3;
                0.0000e-3,   0.0000e-3, 0e-3;
                0.0000e-3,   0.0000e-3, 400e-3;
                0.0000e-3,   0.0000e-3, 0e-3;
                0.0000e-3,   0.0000e-3, 126e-3];
           
    
%                [0.0000e-3,   0.0000e-3, 152.5000e-3;
%                 0.0000e-3,   0.0000e-3, 187.5000e-3;
%                 0.0000e-3,   0.0000e-3, 212.5000e-3;
%                 0.0000e-3,   0.0000e-3, 187.5000e-3;
%                 0.0000e-3,   0.0000e-3, 212.5000e-3;
%                 0.0000e-3,   0.0000e-3, 187.5000e-3;
%                 0.0000e-3,   0.0000e-3, 126.0000e-3];

%                 [ 0.0000e-3, 0.0000e-3, 152.5000e-3;
%                 0.0000e-3,   -11.0000e-3, 187.5000e-3;
%                 0.0000e-3,   +11.0000e-3, 212.5000e-3;
%                 0.0000e-3,   +11.0000e-3, 187.5000e-3;
%                 0.0000e-3,   -11.0000e-3, 212.5000e-3;
%                 0.0000e-3, -62.0000e-3, 187.5000e-3;
%                 0.0000e-3, +62.0000e-3,  126e-3];     


            
            JointAxes = [0,0,1;
                0,1,0;
                0,0,1;
                0,-1,0;
                0,0,1;
                0,1,0;
                0,0,1];
            
            T_finalLink_Flange = sym(eye(4));
            T_finalLink_Flange(1:3,4) = [0,0,35e-3]';

T_ij = sym(zeros(4,4,noJoints+1));

for curJoint = 1:noJoints
    
        angles = JointAxes(curJoint,:)*q(curJoint);
        distance = Lengths(curJoint,:);
        T_ij(:,:,curJoint) = func_make_translation(distance)*func_make_rotation('xyz', angles);    

end

T_End_Flange = T_finalLink_Flange;

T_0j = sym(zeros(4,4,noJoints+1));
T_0j(:,:,1) = T_ij(:,:,1);
T_ij(:,:,noJoints+1)= T_End_Flange;   
for i = 2:noJoints
    T_0j(:,:,i) = T_0j(:,:,i-1)*T_ij(:,:,i);
end

T_0j(:,:,end) = T_0j(:,:,end-1)*T_End_Flange;