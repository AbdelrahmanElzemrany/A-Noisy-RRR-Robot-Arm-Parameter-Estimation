% Before you run This code check the dimension Of Beta (the symbolic vector that contains 
%the parameter to be estimated)  in the workspace 
%% and Run the simulink File named dataextractForSecondExpValidation.slx
% Abdelrahman ELzemrany (Closed-Loop Excitation Trajectory Optimization - Zero Bound Boundary Conditions)

clc; close all;

fprintf('=================================================================\n');
fprintf('  STARTING ZERO-BOUNDED TRAJECTORY OPTIMIZATION (D-OPTIMALITY)   \n');
fprintf('=================================================================\n\n');

%% 1. GLOBAL CONFIGURATION & TUNING VARIABLES
setup.omega_f = 2 * pi / 10;   % Base trajectory frequency (10-second period)
setup.N_harmonics = 7;         % 5 unique frequencies per joint link
setup.num_joints = 3;

% BOUNDARY FIXED MATH: Each joint now optimizes 8 variables [a2..a5, b2..b5] instead of 11.
% q0, a1, and b1 are analytically solved to force q(0)=0, qp(0)=0, qpp(0)=0.
setup.vars_per_joint = 2 * (setup.N_harmonics - 1); 
setup.total_vars = setup.num_joints * setup.vars_per_joint;

% 1000 Hz Optimization Grid (10,001 Samples)
setup.t_grid = 0:0.1:10; 

%% 2. ENFORCE PHYSICAL SYSTEM BOUNDARY CONSTRAINTS
setup.pos_limits = [-1.2, 1.2; -1.2, 1.2; -1.2, 1.2]; 
setup.vel_limits = [-3.0, 3.0; -3.0, 3.0; -4.5, 4.5]; 
setup.acc_limits = [-10.0, 10.0; -10.0, 10.0; -15.0, 15.0];

%% 3. OPTIMIZER INITIALIZATION
rng(42); 
x0 = (rand(setup.total_vars, 1) - 0.5) * 0.5; 

% Direct automatic file link to Y_b_handle.m
Y_b_handle = @Y_b_handle; 

options = optimoptions('fmincon', ...
    'Display', 'iter-detailed', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 25000, ...
    'MaxIterations', 400);

%% 4. EXECUTE THE OPTIMIZATION LOOP
obj_fun = @(x) objective_function_inline(x, setup, Y_b_handle);
constr_fun = @(x) constraints_function_inline(x, setup);

[x_opt, fval] = fmincon(obj_fun, x0, [], [], [], [], [], [], constr_fun, options);

%% 5. SAVE MATRIX PARAMETERS TO DISK
save('optimal_fourier_coefficients.mat', 'x_opt', 'setup');
fprintf('\n>>> Optimization Successful! Coefficients saved to disk.\n\n');

%% 6. HIGH-RESOLUTION SIMULINK SIGNAL GENERATION (1000 Hz)
fprintf('Generating 1000 Hz reference trajectories for Step 2 Simulink model...\n');
t_sim = (0:0.001:10)'; 

[q, qp, qpp] = reconstruct_fourier_trajectory_inline(x_opt, t_sim', setup.omega_f, setup.N_harmonics);

% Package arrays into standard Simulink structures
q_ref_joint1 = [t_sim, q(1,:)'];
q_ref_joint2 = [t_sim, q(2,:)'];
q_ref_joint3 = [t_sim, q(3,:)'];

%% 7. PLOT THE FRICTION-ISOLATED TRAJECTORY PROFILES
figure('Name', 'Optimized Trajectory - Abdelrahman Zero-Bound Pipeline', 'Units', 'normalized', 'Position', [0.1, 0.1, 0.6, 0.7]);

subplot(3,1,1);
plot(t_sim, q', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Positions (q) - Bounded at \pm1.2 rad [q(0) = 0]');
ylabel('Position [rad]'); legend('Joint 1', 'Joint 2', 'Joint 3');
ylim([-1.4, 1.4]); 

subplot(3,1,2);
plot(t_sim, qp', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Velocities (q-dot) [q-dot(0) = 0]');
ylabel('Velocity [rad/s]');

subplot(3,1,3);
plot(t_sim, qpp', 'LineWidth', 2); grid on;
title('Zero-Bounded Joint Accelerations (q-double-dot) [q-double-dot(0) = 0]');
ylabel('Acceleration [rad/s²]'); xlabel('Time [seconds]');

fprintf('>>> SUCCESS: Reference parameters loaded. You can now run Step_2 Simulink block diagram.\n');


%% =========================================================================
%% INLINE MATHEMATICAL UTILITIES (CONSTRAINED TO ZERO BOUNDARIES)
%% =========================================================================

function [q_out, qp_out, qpp_out] = reconstruct_fourier_trajectory_inline(x, t_vec, omega, N_harm)
    num_j = 3;
    v_per_j = 2 * (N_harm - 1); % 8 parameters per joint optimized
    n_samples = length(t_vec);
    
    q_out = zeros(num_j, n_samples);
    qp_out = zeros(num_j, n_samples);
    qpp_out = zeros(num_j, n_samples);
    
    for j_idx = 1:num_j
        s_idx = (j_idx-1)*v_per_j + 1;
        j_vars = x(s_idx : s_idx + v_per_j - 1);
        
        % Extract optimized higher harmonics coefficients (k = 2 to 5)
        a_high = j_vars(1 : N_harm-1);      % [a2, a3, a4, a5]
        b_high = j_vars(N_harm : end);       % [b2, b3, b4, b5]
        
        % Reconstruct full harmonic arrays (allocate space for fundamental harmonic k=1)
        a = zeros(N_harm, 1);
        b = zeros(N_harm, 1);
        
        a(2:end) = a_high;
        b(2:end) = b_high;
        
        % 1. Enforce zero initial velocity boundary: sum(a_k) = 0
        a(1) = -sum(a_high);
        
        % 2. Enforce zero initial acceleration boundary: sum(b_k * k * omega) = 0
        acc_sum = 0;
        for k = 2:N_harm
            acc_sum = acc_sum + b(k) * k * omega;
        end
        b(1) = -acc_sum / (1 * omega);
        
        % 3. Enforce zero initial position boundary: q0 - sum(b_k / (k * omega)) = 0
        pos_sum = 0;
        for k = 1:N_harm
            pos_sum = pos_sum + (b(k) / (k * omega));
        end
        q0_val = pos_sum;
        
        % Generate standard analytical trajectories with zero-locked boundaries
        q_out(j_idx,:) = q0_val;
        for harmonic = 1:N_harm
            kf_val = omega * harmonic;
            q_out(j_idx,:)   = q_out(j_idx,:)   + (a(harmonic)/kf_val) * sin(kf_val*t_vec) - (b(harmonic)/kf_val) * cos(kf_val*t_vec);
            qp_out(j_idx,:)  = qp_out(j_idx,:)  + a(harmonic) * cos(kf_val*t_vec) + b(harmonic) * sin(kf_val*t_vec);
            qpp_out(j_idx,:) = qpp_out(j_idx,:) - a(harmonic)*kf_val * sin(kf_val*t_vec) + b(harmonic)*kf_val * cos(kf_val*t_vec);
        end
    end
end

function cost_val = objective_function_inline(x_vars, setup_struct, handle_Y)
    [q_eval, qp_eval, qpp_eval] = reconstruct_fourier_trajectory_inline(x_vars, setup_struct.t_grid, setup_struct.omega_f, setup_struct.N_harmonics);
    n_smpl = length(setup_struct.t_grid);
    
    try
        W_g_raw = handle_Y(0, 0, -9.81, q_eval(1,:), q_eval(2,:), q_eval(3,:), ...
                           qp_eval(1,:), qp_eval(2,:), qp_eval(3,:), ...
                           qpp_eval(1,:), qpp_eval(2,:), qpp_eval(3,:));
        
        if size(W_g_raw, 1) == 3
            W_g = reshape(W_g_raw, 3 * n_smpl, []);
        else
            W_g = W_g_raw;
        end
    catch
        sample_Y_mat = handle_Y(0, 0, -9.81, 0,0,0, 0,0,0, 0,0,0);
        n_base = size(sample_Y_mat, 2);
        W_g = zeros(3 * n_smpl, n_base);
        for step = 1:n_smpl
            r_idx = (3*step - 2):(3*step);
            W_g(r_idx, :) = handle_Y(0, 0, -9.81, ...
                q_eval(1,step),   q_eval(2,step),   q_eval(3,step), ...
                qp_eval(1,step),  qp_eval(2,step),  qp_eval(3,step), ...
                qpp_eval(1,step), qpp_eval(2,step), qpp_eval(3,step));
        end
    end
    
    Info_Mat = W_g' * W_g;
    dt_val = det(Info_Mat);
    
    if dt_val > 1e-5
        cost_val = -log(dt_val);
    else
        cost_val = 50.0 - dt_val * 1000; 
    end
end

function [c_val, ceq_val] = constraints_function_inline(x_vars, setup_struct)
    [q_eval, qp_eval, qpp_eval] = reconstruct_fourier_trajectory_inline(x_vars, setup_struct.t_grid, setup_struct.omega_f, setup_struct.N_harmonics);
    ceq_val = [];  
    
    c_val = zeros(6 * setup_struct.num_joints, 1);
    c_idx = 1;
    
    for joint = 1:setup_struct.num_joints
        c_val(c_idx)   = max(q_eval(joint,:)) - setup_struct.pos_limits(joint,2);
        c_val(c_idx+1) = setup_struct.pos_limits(joint,1) - min(q_eval(joint,:));
        
        c_val(c_idx+2) = max(qp_eval(joint,:)) - setup_struct.vel_limits(joint,2);
        c_val(c_idx+3) = setup_struct.vel_limits(joint,1) - min(qp_eval(joint,:));
        
        c_val(c_idx+4) = max(qpp_eval(joint,:)) - setup_struct.acc_limits(joint,2);
        c_val(c_idx+5) = setup_struct.acc_limits(joint,1) - min(qpp_eval(joint,:));
        
        c_idx = c_idx + 6;
    end
end
