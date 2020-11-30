function s_x_beta = unit_vector_sx(f_h, s_x, gamma, beta)
A=[];
b=[];
Aeq=[s_x';f_h'];
beq=[norm(s_x)*norm(s_x)*cos(beta);
    norm(s_x)*norm(f_h)*cos(gamma-beta)];
s0=s_x;
fun=@(s) norm(f_h-s);
s = fmincon(fun,s0,A,b,Aeq,beq);
s_x_beta=s/norm(s);

