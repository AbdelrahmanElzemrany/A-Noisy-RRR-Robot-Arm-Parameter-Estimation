% Before you run This code check the dimension Of Beta (the symbolic vector that contains 
%the parameter to be estimated)  in the workspace 
%% and Run the simulink File named dataextractForSecondExpValidation.slx
% Abdelrahman ELzemrany (Position & Torque Only - Noise Filtering Parameter Estimation)

k = 21;      % Dimension of Beta
time = 10;   % Experiment time
T = 0.01;   % Sampling period
l = round(time/T);  % Number of samples (10000)
fs = 1 / T;  % Sampling frequency (1000 Hz)

%% 1. Joint Data Extraction (Raw Noisy Sensors Only)
q1_raw = pos.signals(1).values(1:l); q1_raw = q1_raw(:);
q2_raw = pos.signals(2).values(1:l); q2_raw = q2_raw(:);
q3_raw = pos.signals(3).values(1:l); q3_raw = q3_raw(:);

tau1_raw = torque.signals(1).values(1:l); tau1_raw = tau1_raw(:);  
tau2_raw = torque.signals(2).values(1:l); tau2_raw = tau2_raw(:);
tau3_raw = torque.signals(3).values(1:l); tau3_raw = tau3_raw(:);

%% 2. High-Fidelity Signal Processing Pipeline (Noise Isolation)
fprintf('Filtering raw measurement channels and synthesizing kinematics...\n');

fc = 1; 
[b, a] = butter(4, fc / (fs/2), 'low');

q1 = filtfilt(b, a, q1_raw);
q2 = filtfilt(b, a, q2_raw);
q3 = filtfilt(b, a, q3_raw);

%% 3. Numerical State Synthesis & Post-Derivative Filtering
fprintf('Computing raw derivatives and filtering kinematics...\n');

qp1 = gradient(q1, T);
qp2 = gradient(q2, T); 
qp3 = gradient(q3, T);

qpp1 = gradient(qp1, T);
qpp2 = gradient(qp2, T);
qpp3 = gradient(qp3, T);

%% 3.6 Data Cropping & Torque Filtering Configuration
crop_idx = 100;
valid_range = (crop_idx + 1) : (l - crop_idx);

q1 = q1(valid_range); q2 = q2(valid_range); q3 = q3(valid_range);
qp1 = qp1(valid_range); qp2 = qp2(valid_range); qp3 = qp3(valid_range);
qpp1 = qpp1(valid_range); qpp2 = qpp2(valid_range); qpp3 = qpp3(valid_range);
fc=5;
[b, a] = butter(4, fc / (fs/2), 'low');
tau1_raw = tau1_raw(valid_range);
tau2_raw = tau2_raw(valid_range);
tau3_raw = tau3_raw(valid_range);

tau1_filt = filtfilt(b, a, tau1_raw);
tau2_filt = filtfilt(b, a, tau2_raw);
tau3_filt = filtfilt(b, a, tau3_raw);

% --- EXPERIMENTAL TOGGLE ---
use_filtered_torque = false; 

if use_filtered_torque
    fprintf('Configuring target vector with FILTERED torque data...\n');
    tt = [tau1_filt; tau2_filt; tau3_filt];
else
    fprintf('Configuring target vector with RAW torque data...\n');
    tt = [tau1_raw; tau2_raw; tau3_raw];
end

l = length(valid_range);

%% Rearranging The Observation Regressor Matrix 
fprintf('Assembling Regressor Matrix...');
Yb = zeros(3, k, l);
for i = 1:l
   Yb(:,:,i) = Y_b_handle(0,0,-9.8, q1(i),q2(i),q3(i), qp1(i),qp2(i),qp3(i), qpp1(i),qpp2(i),qpp3(i));
end

% SPEEDUP: Vectorized Matrix Rearrangement
sum1 = squeeze(Yb(1, :, :))';
sum2 = squeeze(Yb(2, :, :))';
sum3 = squeeze(Yb(3, :, :))';
Yc = [sum1; sum2; sum3]; 

fprintf(' Done.\n');

%% 4. The Convex constrained optimization 
%% 4.1 Unconstrained Baseline
theta_init = (Yc'*Yc) \ (Yc'*tt); % Replaced pinv with efficient slash operator

%% 4.2 Constrained Optimization Setup
fprintf('Pre-calculating Mass Matrix Grid Regressors for Optimization...\n');

q_lims = [-pi, pi;       
          -pi/2, pi/2;   
          -pi/2, pi/2];  

q1_test = linspace(q_lims(1,1), q_lims(1,2), 5);
q2_test = linspace(q_lims(2,1), q_lims(2,2), 5);
q3_test = linspace(q_lims(3,1), q_lims(3,2), 5);

Y_grid_blocks = cell(5, 5, 5, 3); 

for i = 1:5
    for j = 1:5
        for m = 1:5
            q_curr = [q1_test(i); q2_test(j); q3_test(m)];
            for col = 1:3
                qpp_pulse = zeros(3,1);
                qpp_pulse(col) = 1;
                Y_grid_blocks{i,j,m,col} = Y_b_handle(0,0,0, q_curr(1),q_curr(2),q_curr(3), 0,0,0, qpp_pulse(1),qpp_pulse(2),qpp_pulse(3));
            end
        end
    end
end

fprintf('Running Convex Constrained Optimization via fmincon...\n');

% Dimension-safe objective function
obj_fun = @(theta) sum((Yc * theta(:) - tt(:)).^2);

% --- TUNABLE EPSILON UPPER PARAMETER VALUE ---
epsilon_val = 0.037; % Change this threshold value here as needed

options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'ScaleProblem', 'obj-and-constr', ... 
    'Display', 'iter-detailed', ...        
    'OptimalityTolerance', 1e-6, ...       
    'StepTolerance', 1e-6);

% Passed epsilon_val directly into the function call
[theta, fval, exitflag] = fmincon(obj_fun, theta_init, [], [], [], [], [], [], ...
    @(theta) mass_constraints_fast(theta, Y_grid_blocks, epsilon_val), options);


%% --- OPTIMIZED HELPER CONSTRAINT FUNCTION ---
function [c, ceq] = mass_constraints_fast(theta, Y_grid_blocks, epsilon)
    ceq = []; 
    
    c = zeros(125, 1);
    idx = 1;
    M_curr = zeros(3,3);
    
    for i = 1:5
        for j = 1:5
            for m = 1:5
                M_curr(:, 1) = Y_grid_blocks{i,j,m,1} * theta;
                M_curr(:, 2) = Y_grid_blocks{i,j,m,2} * theta;
                M_curr(:, 3) = Y_grid_blocks{i,j,m,3} * theta;
                
                % FIX: Force structural symmetry to prevent complex eigenvalues
                M_curr = (M_curr + M_curr') / 2;
                
                eg = eig(M_curr);
                c(idx) = epsilon - min(eg);
                idx = idx + 1;
            end
        end
    end
end
