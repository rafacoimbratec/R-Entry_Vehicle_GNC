%% Planet Constants - Mars
clear; clc; close all;
% === Mars Radius ===
Mars_radius = 3396.2e3;  % [m]

% === Mars Gravitational Parameter ===
Mars_mu = 4.282837e13;  % Gravitational parameter for Mars [m^3/s^2]

% Planetary spin rate [rad/s]
Mars_omega = 7.088e-5;         % rad/s (sidereal rotation period ~24h 37m)

% === Mars Atmospheric Parameters ===
%H_s = 11100;   % Scale height of Mars' atmosphere [m]
%rho_0 = 0.020; % Reference density of Mars' atmosphere at surface [kg/m^3]
% Constants
alpha1 = 559.35;
alpha2 = 188.95;

% === Specific Heat Ratio ===
specific_heat = 1.29;  % Specific heat ratio for CO2-dominated Mars atmosphere (Cp/Cv), dimensionless

% === Specific Gas Constant for Mars ===
R_star = 8314.32;   % Universal gas constant [J/(kmol*K)]
M_CO2 = 44.01;      % Molecular mass of CO2 [kg/kmol]
R_s = R_star / M_CO2;  % Specific gas constant for Mars atmosphere [J/(kg*K)]

% === Heat Flux Model ===
N = 0.5;
M = 3.15;
k_heat_flux = 5.3697e-5;

deg2rad = pi/180;

%% Simulation Settings
startTime = 0.0;  % Start time of simulation [s]
stopTime = 7000;  % Stop time of simulation [s]
stepTime = 0.01;  % Time step of simulation [s]

%% Initial Conditions
h0  = 125e3;        % Initial altitude [m]
r0 = (h0+Mars_radius);
theta0 = -176.40167*deg2rad; % Initial longitude [deg]
phi0   = (-21.3)*deg2rad;      % Initial latitude [deg]
V0  = 4700;          % Initial velocity [m/s]
gamma0 = (-10)*deg2rad;        % Initial flight path angle [deg]
psi0   = (-2.8758)*deg2rad;    % Initial heading angle [deg]

%% Final Conditions
theta_t = (-175.8)*deg2rad;   % Target longitude [deg]
phi_t   = (0.276)*deg2rad;    % Target latitude [deg]
V_t     = 450;      % Final velocity [m/s]
h_t     = 2480;     % Final altitude [m]
rt = (h_t+Mars_radius);
e_t = Mars_mu / rt - 0.5 * V_t^2;
%% Targeting Accuracy Requirement
Rgo_target = 5e3;  % Maximum allowable targeting error [m] (5 km)

%% Entry Vehicle Aerodynamic Properties
L_over_D = 0.54;   % Lift-to-drag ratio, dimensionless
beta      = 379;   % Ballistic coefficient [kg/m^2]

%% Path Constraints and Requirements
A_max  = 4;  % Maximum allowable acceleration [m/s^2] (4g)
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

%% Initial Bank Angle Guess and Final Bank Angle
sigma0_guess_deg = 100;        % Initial guess for bank angle [deg]
sigma_f_deg      = 40;          % Final bank angle at target [deg]

% Convert to radians for MATLAB/Simulink
sigma0_guess = sigma0_guess_deg * deg2rad;
sigma_f      = sigma_f_deg * deg2rad;

fprintf('Initial Bank Guess: %.2f deg, Final Bank: %.2f deg\n', ...
    sigma0_guess_deg, sigma_f_deg);
