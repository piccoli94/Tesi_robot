function jplus=J_PseudoInverse(j)
k=0.5;
jplus=j'/(j*j'+eye(3)*k);