clc% 1. Extract time vectors (assuming modern Dataset or Structure with Time format)
% If you get an error here, check if your workspace contains a single structure named 'out'
time_vec = out.Pos.time; 

% 2. Extract the data arrays [Time x 3 Joints]
desired_pos = out.Pos.data;
torque_appl = out.TorqueApplied.data;
joint_error = out.JointPositionError.data;

% 3. Create a professional, clean multi-plot figure
figure('Color', 'w', 'Name', 'CTC Controller Signals');

% --- Subplot 1: Desired Positions ---
subplot(3,1,1);
plot(time_vec, desired_pos, 'LineWidth', 1.5);
grid on;
title('Desired Joint Positions');
ylabel('Position (rad)');
legend('Base', 'Shoulder', 'Wrist', 'Location', 'best');

% --- Subplot 2: Joint Position Errors ---
subplot(3,1,2);
plot(time_vec, joint_error, 'LineWidth', 1.5);
grid on;
title('Joint Position Tracking Errors');
ylabel('Error (rad)');


