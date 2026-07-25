%%This Code For Reforming the estimated inverse dynamics To construct the
%Feedforward-Feedback Controller or The Computed Torque Controller 
%Abdelrahman Elzermany
Ebeta=sym(theta);
EstTau=Y_b*Ebeta ;
syms gx gy gz

%% 1. Isolate Pure Gravity Matrix (Set velocities and accelerations to 0)
EstTau_G = subs(EstTau, [Qp; Qpp; gx; gy; gz], [zeros(6,1); 0; 0; -9.8]);

%% 2. Isolate Pure Coriolis/Centrifugal Matrix 
EstTau_H = subs(EstTau, [Qpp; gx; gy; gz], [zeros(3,1); 0; 0; -9.8]);
EstTau_C = EstTau_H - EstTau_G;

%% 3. Isolate Pure Mass/Inertia Matrix (CORRECTED: Set gravity to 0!)
% Setting gravity to 0 eliminates any weight leaks into the mass matrix
EstTau_M_base = subs(EstTau, [Qp; gx; gy; gz], [zeros(3,1); 0; 0; 0]); 

% Reconstruct the 3x3 inertia matrix column-by-column
EstTau_M = sym(zeros(3,3));
for j = 1:3
    I_col = zeros(3,1);
    I_col(j) = 1; 
    
    % Substitute the symbolic Qpp vector with the numerical unit pulse
    EstTau_M(:,j) = subs(EstTau_M_base, Qpp, I_col);
end

%% 4. Generate the Clean MATLAB Functions
matlabFunction(EstTau_M, 'File', 'get_RRR_M', 'Vars', {Q},'Optimize',false);
matlabFunction(EstTau_C, 'File', 'get_RRR_C', 'Vars', {Q, Qp},'Optimize',false);
matlabFunction(EstTau_G, 'File', 'get_RRR_G', 'Vars', {Q},'Optimize',false);
