%% func_getLambda.m
% CIAO DANILO
function Lambda = func_getLambda(J,A)
% dbstop if warning

Ainv = A\eye(size(A));
LambdaInv = J*Ainv*J';
Lambda = LambdaInv\eye(size(LambdaInv));