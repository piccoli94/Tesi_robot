%% func_getInverseJacobian.m
% The function calculates the generalized inverse Jacobian
function Jbar = func_getInverseJacobian(J,A)

Ainv = A\eye(size(A));
Lambda = func_getLambda(J,A);
Jbar = Ainv*J'*Lambda;