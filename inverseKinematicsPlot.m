function T_traj = inverseKinematicsPlot(desired)

Jacobian = findJacobian();

% initial estimate q0
q0 = [0;0;0;0];
T0 = forwardKinematics(q0);
error = desired - T0;
q = q0;
i = 1;
T_traj = [];
while norm(error) > 0.1
    theta_1 = q(1);
    theta_2 = q(2);
    theta_3 = q(3);
    theta_4 = q(4);
    digitsOld = digits(5);
    J_temp = vpa(subs(Jacobian));
    T = forwardKinematics(q);
    error = desired - T;
    dq = pinv(J_temp)*error*0.5;
    q = q+dq;
    T_traj(:,i) = T;
    i = i+1;
end

%angle = vpa(mod(q_traj,2*pi));

