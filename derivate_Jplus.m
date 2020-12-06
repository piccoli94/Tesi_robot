function djplus=derivate_Jplus(j,dj_part)

if dim(j(:,1))==3
    I=eye(3);
elseif dim(j(:,1))==6
    I=eye(6);
end
jplus=pinv(j);
djplus_q1= -(jplus * dj_part(:,:,1) * jplus) + (jplus * jplus' * dj_part(:,:,1)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,1)' * jplus' * jplus);
djplus_q2= -(jplus * dj_part(:,:,2) * jplus) + (jplus * jplus' * dj_part(:,:,2)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,2)' * jplus' * jplus);
djplus_q3= -(jplus * dj_part(:,:,3) * jplus) + (jplus * jplus' * dj_part(:,:,3)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,3)' * jplus' * jplus);
djplus_q4= -(jplus * dj_part(:,:,4) * jplus) + (jplus * jplus' * dj_part(:,:,4)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,4)' * jplus' * jplus);
djplus_q5= -(jplus * dj_part(:,:,5) * jplus) + (jplus * jplus' * dj_part(:,:,5)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,5)' * jplus' * jplus);
djplus_q6= -(jplus * dj_part(:,:,6) * jplus) + (jplus * jplus' * dj_part(:,:,6)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,6)' * jplus' * jplus);
djplus_q7= -(jplus * dj_part(:,:,7) * jplus) + (jplus * jplus' * dj_part(:,:,7)' * (I - j * jplus)) + ((I - jplus * j) * dj_part(:,:,7)' * jplus' * jplus);

djplus=djplus_q1+djplus_q2+djplus_q3+djplus_q4+djplus_q5+djplus_q6+djplus_q7;