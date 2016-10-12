function theta = inverseKinematics(desired)

Jacobian = findJacobian();

% initial estimate q0
q0 = [0;0;0;0];
T0 = forwardKinematics(q0);
error = desired - T0;
q = q0;

while norm(error) > 1    
    theta_1 = q(1);
    theta_2 = q(2);
    theta_3 = q(3);
    theta_4 = q(4);
    digitsOld = digits(5);
    J_temp = vpa(subs(Jacobian));
    dq = pinv(J_temp)*error;
    q = q+dq;
    T = forwardKinematics(q);
    error = desired - T;
end

theta = vpa(mod(q,2*pi));

