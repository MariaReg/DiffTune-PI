% define the controller here

% Inputs:
%   theta_r: load position reference
%   omega_r: motor velocity reference
%   omega_dot_r
%   u: torque command

% States include
% omega_m: Motor angular velocity
% omega_l: Load angular velocity
% theta_m: motor angular position
% theta_l: load angular position
% X = [omega_m; omega_l; theta_m; theta_l]
% Xref (ini) = [omega_m; omega_l; theta_m; theta_r]
% ud: containing motor/load angular velocity/position (4outputs)

function [ud, theta_l_integ] = controller(X, Xref, k_vec, theta_r_dot, theta_r_2dot, theta_r_integ, param, dt, theta_l_integ)

    % Controller gains
    k_pos = k_vec(1);
    k_vel = k_vec(2);
    tau_i = k_vec(3);

    % Parameters
    N = param(1);
    J_m = param(2);
    % tau_i = k_vel/k_i;

    % States
    omega_m = X(1);
    omega_l = X(2);
    theta_m = X(3);
    theta_l = X(4);
    theta_r = Xref;



    % Controllers
 
    % P-controller 
    omega_r = k_pos * (theta_r - theta_l) + N * theta_r_dot;
    omega_r_dot = k_pos * (theta_r_dot - omega_l) + N * theta_r_2dot;
    
   
    % PI-controller

    theta_l_integ = theta_l_integ + theta_l * dt;
    omega_r_integ = k_pos * (theta_r_integ - theta_l_integ) + N * theta_r;

    % u = int((omega_r - omega_m) * k_vel * 1/tau_i) + (omega_r - omega_m) * k_vel + omega_r_dot * J_m    
    u = ((omega_r_integ - theta_m) * k_vel * 1/tau_i) + (omega_r - omega_m)* k_vel + omega_r_dot * J_m;
   
    ud = u;

end