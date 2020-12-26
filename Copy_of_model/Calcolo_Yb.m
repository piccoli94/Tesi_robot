load('tau')

syms ml1 ml2 ml3 ml4 ml5 ml6 ml7 mm1 mm2 mm3 mm4 mm5 mm6 mm7 Im1 Im2 Im3 Im4 Im5 Im6 Im7 
syms Il1_1 Il1_2 Il1_3  Il1_4 Il1_5 Il1_6 Il1_7 Il2_1 Il2_2 Il2_3 Il2_4 Il2_5 Il2_6 Il2_7 Il3_1 Il3_2 Il3_3 Il3_4 Il3_5 Il3_6 Il3_7
syms q1 q2 q3 q4 q5 q6 q7 
var=[ml1;ml2;ml3;ml4;ml5;ml6;ml7;mm1;mm2;mm3;mm4;mm5;mm6;mm7;Im1;Im2;Im3;Im4;Im5;Im6;Im7;Il1_1;Il1_2;Il1_3;Il1_4;Il1_5;Il1_6;Il1_7;Il2_1;Il2_2;Il2_3;Il2_4;Il2_5;Il2_6;Il2_7;Il3_1;Il3_2;Il3_3;Il3_4;Il3_5;Il3_6;Il3_7];
% a=Blp;
% [Alp,b]=equationsToMatrix(a==0,var(1:7));
% 
% a=Blo;
% [Alo,b]=equationsToMatrix(a==0,var(22:end));
% 
% a=Bmp;
% [Amp,b]=equationsToMatrix(a==0,var(8:14));
% 
% a=Bmo;
% [Amo,b]=equationsToMatrix(a==0,var(15:21));
% 
% a=B;
% [A,b]=equationsToMatrix(a==0,var);

[Y,b]=equationsToMatrix(tau==0,var);

save('Y_b','Y');