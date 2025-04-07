

% Constants
Mars_radius = 3389.5e3; % [m]

%% Initial Conditions

% Translation State
V_0 = 11e3; % [m/s] - Relative linear velocity
gamma_0 = deg2rad(-9.536); % [rad] - Flight-path angle
chi_0 = deg2rad(0); % [rad] - Heading

R_0 = 220e3 + Mars_radius; % [m] - Position radius
tau_0 = deg2rad(0); % [rad] - Longitude
lambda_0 = deg2rad(0); % [rad] - Latitude

% Attitude State
p_0 = deg2rad(0); % [rad/s] - Roll rate
q_0 = deg2rad(0); % [rad/s] - Pitch rate
r_0 = deg2rad(0); % [rad/s] - Yaw rate

alpha_0 = deg2rad(0); % [rad] - Angle of attack
beta_0 = deg2rad(0); % [rad] - Angle of sideslip
sigma_0 = deg2rad(0); % [rad] - Bank angle

% Initial mass
m_0 = 4976; % [kg]

% Initial moments of inertia
I_xx = 5617.61; % [kg/m^2]
I_yy = 4454.62; % [kg/m^2]
I_zz = 4454.80; % [kg/m^2]

% Initial products of inertia
I_xy = 0; % [kg/m^2]
I_yz = 0; % [kg/m^2]
I_xz = 1752.00; % [kg/m^2]
