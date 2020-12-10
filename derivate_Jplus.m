function djplus=derivate_Jplus(J,dj)
I1=eye(3);
I2=eye(7);
% if max(size(J(:,1)))==3
%     I=eye(3);
% elseif max(size(J(:,1)))==6
%     I=eye(6);
% end
jplus=J'/(J*J' + eye(3)*0.1);
djplus= -(jplus * dj * jplus) + (jplus * jplus' * dj' * (I1 - J * jplus)) + ((I2 - jplus * J) * dj' * jplus' * jplus);
