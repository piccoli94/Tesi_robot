%==========================================================================
% HELP FUNCTION 3 - CALC TRANS MATRIX 4x4 or 3x3 for Rotation
%==========================================================================
function T = func_make_rotation(  Order, Angle  )
    %======================================================================   
     switch (Order),
         case 'xyz',          
           T=[                                           cos(Angle(2))*cos(Angle(3)),                                           -cos(Angle(2))*sin(Angle(3)),                 sin(Angle(2)),     0;
               sin(Angle(1))*sin(Angle(2))*cos(Angle(3))+cos(Angle(1))*sin(Angle(3)), -sin(Angle(1))*sin(Angle(2))*sin(Angle(3))+cos(Angle(1))*cos(Angle(3)),  -sin(Angle(1))*cos(Angle(2)),     0;
              -cos(Angle(1))*sin(Angle(2))*cos(Angle(3))+sin(Angle(1))*sin(Angle(3)),  cos(Angle(1))*sin(Angle(2))*sin(Angle(3))+sin(Angle(1))*cos(Angle(3)),   cos(Angle(1))*cos(Angle(2)),     0;
                                                                                   0,                                                                      0,                             0,     1];
          case 'zyx',
            T=[cos(Angle(3))*cos(Angle(2)), -sin(Angle(3))*cos(Angle(1))+cos(Angle(3))*sin(Angle(2))*sin(Angle(1)),  sin(Angle(3))*sin(Angle(1))+cos(Angle(3))*sin(Angle(2))*cos(Angle(1)),      0;
               sin(Angle(3))*cos(Angle(2)),  cos(Angle(3))*cos(Angle(1))+sin(Angle(3))*sin(Angle(2))*sin(Angle(1)), -cos(Angle(3))*sin(Angle(1))+sin(Angle(3))*sin(Angle(2))*cos(Angle(1)),      0;
                            -sin(Angle(2)),                                            cos(Angle(2))*sin(Angle(1)),                                            cos(Angle(2))*cos(Angle(1)),      0;
                                         0,                                                                      0,                                                                      0,      1];
                                     
     end;    
return;
