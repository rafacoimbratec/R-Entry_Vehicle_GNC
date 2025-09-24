%% Planet Constants - Mars

% === Mars Radius ===
Mars_radius = 3389.5e3;  % [m]

% === Mars Gravitational Parameter ===
Mars_mu = 4.282837e13;  % Gravitational parameter for Mars [m^3/s^2]

% === Mars Atmospheric Parameters ===
H_s = 11100;   % Scale height of Mars' atmosphere [m]
rho_0 = 0.020; % Reference density of Mars' atmosphere at surface [kg/m^3]

% === Specific Heat Ratio ===
specific_heat = 1.29;  % Specific heat ratio for CO2-dominated Mars atmosphere (Cp/Cv), dimensionless

% === Specific Gas Constant for Mars ===
R_star = 8314.32;   % Universal gas constant [J/(kmol*K)]
M_CO2 = 44.01;      % Molecular mass of CO2 [kg/kmol]
R_s = R_star / M_CO2;  % Specific gas constant for Mars atmosphere [J/(kg*K)]

%% Simulation Settings
startTime = 0.0;  % Start time of simulation [s]
stopTime = 1200;  % Stop time of simulation [s]
stepTime = 0.01;  % Time step of simulation [s]

%% Initial Conditions
h0  = 125e3;        % Initial altitude [m]
theta0 = -176.40167; % Initial longitude [deg]
phi0   = -21.3;      % Initial latitude [deg]
V0  = 4700;          % Initial velocity [m/s]
gamma0 = -10;        % Initial flight path angle [deg]
psi0   = -2.8758;    % Initial heading angle [deg]

%% Final Conditions
theta_t = -175.8;   % Target longitude [deg]
phi_t   = 0.276;    % Target latitude [deg]
V_t     = 450;      % Final velocity [m/s]
h_t     = 2480;     % Final altitude [m]

%% Targeting Accuracy Requirement
Rgo_target = 5e3;  % Maximum allowable targeting error [m] (5 km)

%% Entry Vehicle Aerodynamic Properties
L_over_D = 0.54;   % Lift-to-drag ratio, dimensionless
beta      = 379;   % Ballistic coefficient [kg/m^2]

%% Path Constraints and Requirements
A_max  = 4 * 9.80665;  % Maximum allowable acceleration [m/s^2] (4g)
q_max  = 13e3;         % Maximum allowable dynamic pressure [Pa] (13 kPa)
Qdot_max = 500e3;      % Maximum allowable heat rate [W/m^2] (500 kW/m^2)

%% Compute Initial Range-to-Go

% Convert degrees to radians
deg2rad = pi/180;
lat0   = -21.3 * deg2rad;
lon0   = -176.40167 * deg2rad;
lat_t  = 0.276 * deg2rad;
lon_t  = -175.8 * deg2rad;

% Use the spherical_metrics function
[d, sin_d, cos_d, RC, RD, RD_go, Rgo] = spherical_metrics(lat0, lon0, lat0, lon0, lat_t, lon_t, Mars_radius);

% Range-to-go is simply great-circle distance to target (d)
fprintf('Initial Range-to-Go: %.2f km\n', Rgo/1000);
