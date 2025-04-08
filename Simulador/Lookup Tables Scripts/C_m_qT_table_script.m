% Create Apollo pitch-damping moment coefficient lookup table data for Simulink
% Based on Table B.4 from Mosely et al. 1967

% Mach numbers (first column in the table)
Mach_numbers_Cm_q = [2.5; 3.0; 4.0; 6.0; 10.2];

% Angles of attack (first row in the table)
alpha_T_Cm_q = [0, 10, 20, 30, 40];

% Pitch-damping moment coefficient data (Cm_qT) in s/radian
Cm_qT = [...
    -0.6, -0.6, -0.6, -0.6, -0.4;...
    -0.6, -0.6, -0.6, -0.6, -0.4;...
    -0.6, -0.6, -0.6, -0.6, -0.4;...
    -0.6, -0.6, -0.8, -1.6, -0.4;...
    -0.4, -0.4, -0.4, -0.4, -0.4];

% Save the data to a .mat file
save('ApolloPitchDampingCoeff.mat', 'Mach_numbers_Cm_q', 'alpha_T_Cm_q', 'Cm_qT');

% Display confirmation
disp('Apollo pitch-damping coefficient data saved to ApolloPitchDampingCoeff.mat');
disp('To use in Simulink:');
disp('1. Load the .mat file in your model workspace');
disp('2. Use a 2D Lookup Table block');
disp('3. Set the Row index to "Mach_numbers"');
disp('4. Set the Column index to "alpha_T"');
disp('5. Set the Table data to "Cm_qT"');
disp(' ');
disp('Note: Connect Mach number to u1 (top input) and angle of attack to u2 (bottom input)');
disp('Negative values indicate pitch damping (stabilizing moment)');
disp('Units are in seconds per radian (s/rad)');