% Create Apollo pitch-moment coefficient lookup table data for Simulink
% Based on Table B.3 from NAA 1965

% Mach numbers (first column in the table)
Mach_numbers_Cm = [0.90; 1.10; 1.20; 1.35; 1.65; 2.00; 2.40; 3.00; 4.00; 6.00; 10.00];

% Angles of attack (first row in the table)
alpha_T_Cm = [0, 5, 10, 15, 20, 25, 30, 35, 40];

% Pitch-moment coefficient data (Cm_T)
Cm_T = [...
    0.0002, 0.0108, 0.0106, 0.0063, 0.0063, 0.0068, 0.0102, 0.0124, 0.0221;...
    0.0000, 0.0004, 0.0028, 0.0025, -0.0011, 0.0019, 0.0082, 0.0068, -0.0166;...
    0.0001, 0.0166, 0.0137, 0.0098, 0.0019, -0.0021, -0.0086, -0.0168, -0.0440;...
    -0.0003, 0.0075, 0.0089, 0.0023, -0.0012, -0.0038, -0.0076, -0.0452, -0.0806;...
    0.0003, 0.0022, 0.0085, 0.0109, 0.0051, -0.0046, -0.0264, -0.0574, -0.0868;...
    -0.0001, 0.0105, 0.0105, 0.0050, 0.0009, -0.0136, -0.0382, -0.0591, -0.0766;...
    -0.0002, 0.0018, -0.0056, 0.0021, 0.0018, -0.0151, -0.0306, -0.0522, -0.0700;...
    0.0000, 0.0057, -0.0012, 0.0061, 0.0023, -0.0045, -0.0142, -0.0335, -0.0532;...
    -0.0002, -0.0012, 0.0006, 0.0028, 0.0037, 0.0065, -0.0011, -0.0135, -0.0294;...
    0.0002, 0.0033, 0.0008, -0.0017, 0.0014, 0.0086, 0.0069, -0.0056, -0.0219;...
    0.0001, 0.0040, 0.0024, -0.0030, -0.0006, 0.0039, 0.0014, -0.0087, -0.0228];

% Save the data to a .mat file
save('ApolloPitchMomentCoeff.mat', 'Mach_numbers_Cm', 'alpha_T_Cm', 'Cm_T');

% Display confirmation
disp('Apollo pitch-moment coefficient data saved to ApolloPitchMomentCoeff.mat');
disp('To use in Simulink:');
disp('1. Load the .mat file in your model workspace');
disp('2. Use a 2D Lookup Table block');
disp('3. Set the Row index to "Mach_numbers"');
disp('4. Set the Column index to "alpha_T"');
disp('5. Set the Table data to "Cm_T"');
disp(' ');
disp('Note: Connect Mach number to u1 (top input) and angle of attack to u2 (bottom input)');
disp('Negative values indicate nose-down pitching moments');