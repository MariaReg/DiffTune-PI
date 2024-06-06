% This function defines how the sensitivity propgation runs at each sample
% time

% Inputs
% sensitivity: dxdtheta
% X: actual state, [omega_m; omega_l; theta_m; theta_l]
% Xref: desired/reference state, [omega_m; omega_l; theta_m; theta_r]
% u: torque command (output of controller, input to system)
% J_m: moment of inertia (from motor mechanical parameters)
% theta: controller gains, k_vec = [k1; k2; k_pos]
% dt: sample time (taken from simulation parameter)

% Outputs
% dXdphi: calculated sensitivity
% dudphi: calculated sensitivity

function [dXdphi,dudphi] = sensitivityComputation(sensitivity, X, Xref, ...
    theta_r_dot, u, param, theta, dt)

J_m = param.J_m;

% Evaluate the Jacobians
dfdX = grad_f_X_fcn(X,u,dt,J_m);
dfdX = full(dfdX);    % full() converts sparse matrix to full matrix

dfdu = grad_f_u_fcn(X,u,dt,J_m);
dfdu = full(dfdu);

dhdX = grad_h_X_fcn(X,Xref,theta_r_dot,theta,J_m);
dhdX = full(dhdX);

dhdtheta = grad_h_theta_fcn(X,Xref,theta_r_dot,theta,J_m);
dhdtheta = full(dhdtheta);

% Assemble the Jacobians to compute the sensitivity
dXdphi = (dfdX + dfdu * dhdX) * sensitivity + dfdu * dhdtheta;
dudphi = dhdX * sensitivity + dhdtheta;

end
