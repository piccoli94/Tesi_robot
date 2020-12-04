function tau_g = gravity(q,DynamicParameters)
%#codegen

g = DynamicParameters(22);
%pl1y = DynamicParameters(23);
pl2z = DynamicParameters(24);
pl3y = DynamicParameters(25);
pl4z = DynamicParameters(26);
pl5y = DynamicParameters(27);
pl7z = DynamicParameters(28);
%pm1z = DynamicParameters(29);
pm3z = DynamicParameters(30);
pm5z = DynamicParameters(31);

q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);

s2 = sin(q2);
c2 = cos(q2);
s3 = sin(q3);
c3 = cos(q3);
s4 = sin(q4);
c4 = cos(q4);
s5 = sin(q5);
c5 = cos(q5);
s6 = sin(q6);
c6 = cos(q6);

g1 = 0;
g2 = -(g*(9320*s2 + 3237*c4*s2 + 5000*pl2z*s2 + 5000*pl3y*s2 + 2500*pm3z*s2 + 3237*c2*c3*s4 + 5000*c4*pl4z*s2 + 5000*c4*pl5y*s2 + 2500*c4*pm5z*s2 + 5000*c2*c3*pl4z*s4 + 5000*c2*c3*pl5y*s4 + 250*c4*c6*pl7z*s2 + 2500*c2*c3*pm5z*s4 + 250*c2*c3*c6*pl7z*s4 - 250*c2*pl7z*s3*s5*s6 - 250*c5*pl7z*s2*s4*s6 + 250*c2*c3*c4*c5*pl7z*s6))/2500;
g3 = (g*s2*(3237*s3*s4 + 5000*pl4z*s3*s4 + 5000*pl5y*s3*s4 + 2500*pm5z*s3*s4 + 250*c6*pl7z*s3*s4 + 250*c3*pl7z*s5*s6 + 250*c4*c5*pl7z*s3*s6))/2500;
g4 = (c3*c5*g*pl7z*s2*s4*s6)/10 - 2*c2*g*pl4z*s4 - 2*c2*g*pl5y*s4 - c2*g*pm5z*s4 - (3237*c3*c4*g*s2)/2500 - 2*c3*c4*g*pl4z*s2 - 2*c3*c4*g*pl5y*s2 - (c2*c6*g*pl7z*s4)/10 - c3*c4*g*pm5z*s2 - (c3*c4*c6*g*pl7z*s2)/10 - (c2*c4*c5*g*pl7z*s6)/10 - (3237*c2*g*s4)/2500;
g5 = (g*pl7z*s6*(c5*s2*s3 + c2*s4*s5 + c3*c4*s2*s5))/10;
g6 = -(g*pl7z*(c2*c4*s6 - c3*s2*s4*s6 - c6*s2*s3*s5 + c2*c5*c6*s4 + c3*c4*c5*c6*s2))/10;
g7 = 0;

tau_g = [g1;g2;g3;g4;g5;g6;g7];
