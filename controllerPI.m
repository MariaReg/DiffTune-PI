% ud: containing motor/load angular velocity/position (4outputs)

function ud = controllerPI(X, Xref, k_vec, theta_r_dot, param, time); % t for time

    %parameters
    N = param.N;
    J_m = param.J_m;
    tau_i = 1;

    % Controller gains
    k1 = k_vec(1);
    k_i = k_vec(2);
    k_vel = k_vec(3);

    % States
    omega_m = X(1);
    omega_l = X(2);
    theta_l = X(4);
    theta_r = Xref(4);
    omega_m_dot = dXdt(1);

    % Controllers
 
    % P-controller 
    omega_r = k_pos * (theta_r - theta_l) + N * theta_r_dot;
    omega_r_dot = k_pos * (theta_r_dot - omega_l) + N * theta_r_2dot;
    omega_r_integ = -cos(2*pi*time);
   
    % PI-controller
    %u = int((omega_r - omega_m) * k_vel * 1/tau_i) + (omega_r - omega_m) * k_vel + diff(omega_r) * J_m;

    u = k_pos*(omega_r_integ - theta_m) * k_vel*1/tau_i) + (omega_r - omega_m)* k_vel + omega_r_dot * J_m;
   
    ud = u;

end