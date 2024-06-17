function udP = controllerP(X, Xref, k_pos, theta_r_dot, N) 

    % States
    theta_l = X(4);
    theta_r = Xref;

    % Controller
    omega_r = k_pos * (theta_r - theta_l) + N * theta_r_dot;
    disp('omega_r:) =');
    disp(omega_r);
    udP = omega_r;
end

