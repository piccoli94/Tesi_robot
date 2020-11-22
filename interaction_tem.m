function c_x=interaction_tem(interact_region, beta, sigma, f_h, s_x)
region=interact_region.region;
gamma=interact_region.gamma;
if(strcmp(region,'RR'))
    c_x=f_h;
% elseif(strcmp(region,'PR'))
%     c_x= % da risolvere
elseif(strcmp(region,'CR'))
    c_x=0;
end