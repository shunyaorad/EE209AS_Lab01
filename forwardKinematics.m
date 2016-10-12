function coordinate = forwardKinematics(theta)

pi = sym('pi');

theta_1 = theta(1);
theta_2 = theta(2);
theta_3 = theta(3);
theta_4 = theta(4);
theta_5 = 0;

% modfied D-H parameters
l1 = 0;
l2 = 6;
l3 = 12;
l4 = 12;
l5 = 12;

a_1 = 0;
alpha_1 = 0;
d_1 = 0;

a_2 = l1;
alpha_2 = pi/2;
d_2 = 0;

a_3 = l2;
alpha_3 = 0;
d_3 = 0;

a_4 = l3;
alpha_4 = 0;
d_4 = 0;

a_5 = l4;
alpha_5 = 0;
d_5 = 0;
theta_5 = 0;

T0_1 = findTi(alpha_1, a_1, d_1, theta_1);
T1_2 = findTi(alpha_2, a_2, d_2, theta_2);
T2_3 = findTi(alpha_3, a_3, d_3, theta_3);
T3_4 = findTi(alpha_4, a_4, d_4, theta_4);
T4_5 = findTi(alpha_5, a_5, d_5, theta_5);

T0_5 = T0_1*T1_2*T2_3*T3_4*T4_5;

endEffector = T0_5*[0;0;0;1];

digitsOld = digits(5);
coordinate = vpa(subs(endEffector));
coordinate = coordinate(1:3);
%coordinate = double(coordinate(1:3));

end