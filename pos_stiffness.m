function k_p=pos_stiffness(delta_x, k_1, a, a_h)

f_p= 1 - exp( -a*( norm(delta_x)^2 - a_h^2 ) );
k_p= k_1 * ( max(0,f_p)^2 ) * exp( -a*( norm(delta_x)^2 - a_h^2 ) );