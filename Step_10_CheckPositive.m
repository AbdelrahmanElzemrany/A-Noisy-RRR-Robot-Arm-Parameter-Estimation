%Created By Abdelrahman Elzermany
clc;
%% Mass Matrix Mathematical Audit Script
% This script tests your unconstrained M matrix for positive definiteness.

% 1. Define your robot's physical joint limits (Adjust these to your robot!)
q1_lim = [-1.2, 1.2]; 
q2_lim = [-1.2, 1.2];
q3_lim = [-1.2, 1.2];

% 2. Create a test grid (change steps to 20 or 30 for a denser search)
steps = 30; 
q1_range = linspace(q1_lim(1), q1_lim(2), steps);
q2_range = linspace(q2_lim(1), q2_lim(2), steps);
q3_range = linspace(q3_lim(1), q3_lim(2), steps);

% Initialize tracking variables
min_eigenvalue = Inf;
max_condition_number = 0;
failed_configs = [];
total_points = steps^3;
checked_points = 0;

fprintf('Auditing %d workspace configurations...\n', total_points);

% 3. Loop through the workspace
for i = 1:steps
    for j = 1:steps
        for k = 1:steps
            q_test = [q1_range(i); q2_range(j); q3_range(k)];
            
            % Evaluate your generated function
            M_hat = get_RRR_M(q_test);
            
            % Calculate eigenvalues and condition number
            eg = eig(M_hat);
            cond_num = cond(M_hat);
            
            % Update global minimum eigenvalue found
            if min(eg) < min_eigenvalue
                min_eigenvalue = min(eg);
            end
            
            % Update global worst-case condition number
            if cond_num > max_condition_number
                max_condition_number = cond_num;
            end
            
            % Check for physical inconsistency (Not positive-definite)
            if any(eg <= 1e-5) || isinf(cond_num) || isnan(cond_num)
                failed_configs = [failed_configs; q_test', eg']; %#ok<AGROW>
            end
        end
    end
end

%% 4. Print the Mathematical Verdict
fprintf('\n================ AUDIT RESULTS ================\n');
fprintf('Minimum Eigenvalue found:     %f\n', min_eigenvalue);
fprintf('Worst Condition Number found:  %f\n', max_condition_number);

if isempty(failed_configs)
    fprintf('\nVERDICT: [SAFE] Your unconstrained model is strictly Positive-Definite across the tested workspace!\n');
    fprintf('Your high gains are completely safe from inverse-matrix division spikes.\n');
else
    fprintf('\nVERDICT: [DANGER] Found %d singular or non-positive-definite configurations!\n', size(failed_configs,1));
    fprintf('In these zones, your high gains will multiply by massive numerical spikes and cause instability.\n');
    disp('Sample unsafe joint angles and their eigenvalues:');
    disp(failed_configs(1:min(5, size(failed_configs,1)), 1:3));
end
