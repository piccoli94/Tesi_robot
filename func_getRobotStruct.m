%% func_getRobotStruct.m
% The function either initializes the a MATLAB-root-structure 

function robot = func_getRobotStruct()

% RGB colours
kukaOrange = [1,0.424,0];
grey = [0.5,0.5,0.5];
black = [0,0,0];
red = [1,0,0];

            
            JointTypes = [1,1,1,1,1,1,1]; % 1: revolute, 2: prismatic;
            JointAxes = [0,0,1;
                0,1,0;
                0,0,1;
                0,-1,0;
                0,0,1;
                0,1,0;
                0,0,1];
            
    
            Lengths = ...
                [ 0.0000e-3, 0.0000e-3, 152.5000e-3;
                0.0000e-3,   -11.0000e-3, 187.5000e-3;
                0.0000e-3,   +11.0000e-3, 212.5000e-3;
                0.0000e-3,   +11.0000e-3, 187.5000e-3;
                0.0000e-3,   -11.0000e-3, 212.5000e-3;
                0.0000e-3, -62.0000e-3, 187.5000e-3;
                0.0000e-3, +62.0000e-3,  79.6e-3];
                   %mass link_0
            Masses=[5 ;3.4525 ;3.4821 ;4.05623 ;3.4822 ;2.1633 ;2.3466; 3.129];
            
            
            
            
            
            
            SegmentColor = [kukaOrange;
                kukaOrange;
                kukaOrange;
                kukaOrange;
                kukaOrange;
                kukaOrange;
                kukaOrange];
            
            EndSegmentIndex = 7;
            T_finalLink_Flange = eye(4);
            T_finalLink_Flange(1:3,4) = [0,0,31.4e-3]';
            
            objScaling = 1/1000;    % currently used .obj files are in mm
            
            % create struct
            noJoints = length(JointTypes);
            
            robot.Name = Name;
            robot.Dimension = '3D';
            robot.NoDofs = noJoints;
            robot.JointTypes = JointTypes;
            robot.JointAxes = JointAxes;
            robot.Lengths = Lengths;
            robot.SegmentColor = SegmentColor;
            robot.EndSegmentIndex = EndSegmentIndex;
            robot.T_finalLink_Flange = T_finalLink_Flange;
            robot.ObjScaling = objScaling;
            robot.T = NaN(4,4,noJoints);
            
    
