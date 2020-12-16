function dk_p=derivate_kp(delta_x, d_delta_x, a_h, a, k_1, region)
if (strcmp(region,'H-DR')|| strcmp(region,'SS-SR') || strcmp(region,'TS-SR'))
    dk_p=0;
else
f_p= 1 - exp( -a*( norm( delta_x )^2 - a_h^2 ) );
dexp=-a*exp(-a*(- a_h^2 + abs(delta_x(1))^2 + abs(delta_x(2))^2 + abs(delta_x(3))^2))*(2*abs(delta_x(1))*sign(delta_x(1))*d_delta_x(1) + 2*abs(delta_x(2))*sign(delta_x(2))*d_delta_x(2) + 2*abs(delta_x(3))*sign(delta_x(3))*d_delta_x(3));
df_p=-2*a*exp(-a*(- a_h^2 + abs(delta_x(1))^2 + abs(delta_x(2))^2 + abs(delta_x(3))^2))*(exp(-a*(- a_h^2 + abs(delta_x(1))^2 + abs(delta_x(2))^2 + abs(delta_x(3))^2)) - 1)*(2*abs(delta_x(1))*sign(delta_x(1))*d_delta_x(1) + 2*abs(delta_x(2))*sign(delta_x(2))*d_delta_x(2) + 2*abs(delta_x(3))*sign(delta_x(3))*d_delta_x(3));

dk_p=+ k_1 * ( df_p ) * exp( -a*( norm(delta_x)^2 - a_h^2 ) ) + k_1 * (f_p)^2 * dexp;

    
end