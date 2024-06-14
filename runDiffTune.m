

close all;
clear all;

addpath('mex\');
import casadi.*

%% define the dimensions
dim_state = 4; % dimension of system state
dim_control = 1;  % dimension of control inputs
dim_controllerParameters = 3;  % dimension of controller parameters

%% Video simulation
param1.generateVideo = true;
if param1.generateVideo
    video_obj = VideoWriter('PI-controller.mp4','MPEG-4');
    video_obj.FrameRate = 15;
    open(video_obj);
end

%% Define simulation parameters (e.g., sample time dt, duration, etc)
dt = 0.001;     % 1 kHz
time = 0:dt:10;

%% constant parameters
% Motor mechanical parameters
J_m = 2.81e-4 + 5.5e-4; % kgm^2 -- Moment of inertia
N = 1;                  % -- Gear ratio
% Values of friction and shaft parameters
% Taken from Table 4.3: Summary of calculated friction and shaft parameters
% (page 40, Dimitrios Papageorgiou phd thesis)
% Shaft constants
K_S = 32.94;    % N m rad^(-1)
D_S = 0.0548;   % N m s rad^(-1)
% Coulomb friction
% (assuming T_C is the average of T_C_m and T_C_l)
T_C = (0.0223 + 0.0232) / 2;    % N m
% Static friction
% (assuming T_S is the average of T_S_m and T_S_l)
% T_S = (0.0441 + 0.0453) / 2;    % N m
% Friction constants
b_fr = 0.0016;  % N m s rad^(-1)
J_l = 1; % kgm^2 -- Moment of inertia

param = [N J_m J_l K_S D_S T_C b_fr];


%% Initialize controller gains (must be a vector of size dim_controllerParameters x 1)
% STSMC (in nonlinear controller for omega_m)
k_pos= 10;      % ignored when hand-tuning PI
% k_i = 1.453488372 * 2.45 * 0.99; % use proportional gain from PI controller (k_vel = 1.45*2.45)
k_vel = 10;
k_i = 10;
k_vec = [k_pos; k_vel; k_i];


%% Define desired trajectory if necessary
freq = 1;   % 1 rad/s
theta_r = sin(freq * time);   % theta_r is a sine wave with frequency 1 rad/s
theta_r_dot = freq * cos(freq * time);
theta_r_2dot = -freq^2 * sin(freq * time);
theta_r_integ = - cos(freq * time) / freq;

%% Initialize variables for DiffTune iterations
learningRate = 0.005;  % Calculate  
maxIterations = 100;
itr = 0;

loss_hist = [];  % storage of the loss value in each iteration
rmse_hist = []; % If we want video
param_hist = []; % storage of the parameter value in each iteration
gradientUpdate = zeros(dim_controllerParameters,1); % define the parameter update at each iteration

%% DiffTune iterations
while (1)
    itr = itr + 1;
    fprintf('------------------------\n');
    fprintf('itr = %d \n', itr);

    fprintf('k_vec = \n');
    disp(k_vec);

    % Initialize state
    X_storage = zeros(dim_state,1);
    
    % Initialize sensitivity
    dx_dtheta = zeros(dim_state,dim_controllerParameters);
    du_dtheta = zeros(dim_control,dim_controllerParameters);

    % Initialize loss and gradient of loss
    loss = 0;
    theta_gradient = zeros(1,dim_controllerParameters);

    % Initialize reference state and desired trajectory

    for k = 1 : length(time) - 1
       
        % Load current state and current reference
        X = X_storage(:,end);   % X = [omega_m; omega_l; theta_m; theta_l]
        Xref = theta_r(k);
 
        % Compute the control action
        u = controller(X, Xref, k_vec, theta_r_dot(k), theta_r_2dot(k), theta_r_integ(k), param, dt); 

        % Compute the sensitivity 
        [dx_dtheta, du_dtheta] = sensitivityComputation(dx_dtheta, X, Xref, theta_r_dot(k), theta_r_2dot(k), theta_r_integ(k), u, param, k_vec, dt);

        % Accumulate the loss
        % (loss is the squared norm of the position tracking error (error_theta = theta_r - theta_l))
        loss = loss + (Xref - X(4))^2;

        % Accumulating the gradient of loss w/ respect to controller parameters
        theta_gradient = theta_gradient + 2 * [0 0 0 X(4)-Xref] * dx_dtheta;

        % Integrate the ode dynamics
        [~,sold] = ode45(@(t,X)dynamics(t, X, u, param),[time(k) time(k+1)], X);
        X_storage = [X_storage sold(end,:)'];   % store the new state

        if (k >= 155)
            disp(k);
            disp(theta_gradient);
        end
        if isnan(theta_gradient) | (theta_gradient == - inf)
           disp('k =');
           disp(k);
           fprintf('theta_gradient is NAN. Quit.\n');
           break;
       end

        
    end

    clear global theta_l_integ;
    

    % Compute the RMSE (root-mean-square error)
    RMSE = sqrt(1 / length(time) * loss);

    % Store loss and RMSE
    loss_hist = [loss_hist loss];
    rmse_hist = [rmse_hist RMSE];

    % Update the gradient
    gradientUpdate = - learningRate * theta_gradient;

    % sanity check
    if isnan(gradientUpdate)
       fprintf('gradient is NAN. Quit.\n');
       break;
    end
   

    % Gradient descent
    k_vec = k_vec + gradientUpdate';    % ' used for transposing matrix or vector (k_vec was called theta in diffTune documents)

    % Projection of all parameters to the feasible set
    % We can vary this to match our system
    if any(k_vec < 0.001)
       neg_indicator = (k_vec < 0.001);
       pos_indicator = ~neg_indicator;
       k_vec_default = 0.001 * ones(dim_controllerParameters,1);
       k_vec = neg_indicator.*k_vec_default + pos_indicator.*k_vec_default;
    end

    % store the parameters
    param_hist = [param_hist k_vec];

    % Plotting
    % set(gcf,'Position',[172 120 950 455]);
    set(gcf,'color','w');

    % Position (theta_l) tracking
    subplot(3,3,[1,2;4,5]);
    plot(time,X_storage(4,:),'DisplayName','actual','LineWidth',1.5);
    hold on;
    plot(time,theta_r,'DisplayName','desired','LineWidth',1.5);
    xlabel('time [s]');
    ylabel('\theta_l [rad]');
    grid on;
    h_lgd = legend;
    set(h_lgd,'Position',[0.3811 0.8099 0.1097 0.0846],'FontSize',10);
    set(gca,'FontSize',10);

    % RMSE
    subplot(3,3,[3;6;9]);
    plot(rmse_hist,'LineWidth',1.5);
    hold on;
    grid on;
    stem(length(rmse_hist),rmse_hist(end),'Color',[0 0.4470 0.7410]);

    xlim([0 100]);
    ylim([0 rmse_hist(1)*1.1]);
    text(50,0.3,['iteration = ' num2str(length(rmse_hist))],'FontSize',12);
    xlabel('iterations');
    ylabel('RMSE [rad]');
    set(gca,'FontSize',10);
    plotedit(gca,'on');
    plotedit(gca,'off');

    drawnow;

    % Visualization for movie
    if param1.generateVideo
        frame = getframe(gcf);
        writeVideo(video_obj,frame);
        clf
    end

    % Terminate if the total number of iterations is more than maxIterations
    if itr >= maxIterations
       break;
    end
end

if param1.generateVideo
    close(video_obj);
end

%% Plot trajectory
figure();
plot(time, theta_r,'DisplayName','theta_r');
hold on;
plot(time, X_storage(4,:),'DisplayName','theta_l');
legend;
ylabel('\theta [rad]');

%% Debug session
check_dx_dtheta = sum(isnan(dx_dtheta),'all');
check_du_dtheta = sum(isnan(du_dtheta),'all');
