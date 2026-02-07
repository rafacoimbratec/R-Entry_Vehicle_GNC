%% Planet Constants - Mars
<<<<<<< HEAD
clear; clc; close all;
format long;

%Check Benito Reference
=======

>>>>>>> parent of a4c5db4 (LNPCG implementation)
% === Mars Radius ===
Mars_radius = 3396.2e3;  % [m]

% === Mars Gravitational Parameter ===
Mars_mu = 4.282837e13;  % Gravitational parameter for Mars [m^3/s^2]

% Planetary spin rate [rad/s]
Mars_omega = 7.088e-5;         % rad/s (sidereal rotation period ~24h 37m)

% === Mars Atmospheric Parameters ===
H_s = 11100;   % Scale height of Mars' atmosphere [m]
rho_0 = 0.020; % Reference density of Mars' atmosphere at surface [kg/m^3]

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
M = 3;
Rn = 0.66; % Nose Radius
k_heat_flux = (1.8980e-4)/sqrt(Rn);

deg2rad = pi/180;

%% Simulation Settings
startTime = 0.0;  % Start time of simulation [s]
stopTime = 7000;  % Stop time of simulation [s]
stepTime = 0.01;  % Time step of simulation [s]

%% Initial Conditions
h0  = 128e3;        % Initial altitude [m]
r0 = (h0+Mars_radius);
theta0 = 66.8*deg2rad; % Initial longitude [deg]
phi0   = (20.83)*deg2rad;      % Initial latitude [deg]
V0  = 5500;          % Initial velocity [m/s]
gamma0 = (-15.5)*deg2rad;        % Initial flight path angle [deg]
psi0   = (90)*deg2rad;    % Initial heading angle [deg]
e_0 = Mars_mu / r0 - 0.5 * V0^2;
%% Final Conditions Mars reachable plots
theta_t = 77.2821*deg2rad; %-176.341*deg2rad;%(-176.392)*deg2rad;   % Target longitude [deg]
phi_t   = 21.0329*deg2rad;%-10.745*deg2rad; %(-4.887)*deg2rad;    % Target latitude [deg]
V_t     = 475;      % Final velocity [m/s]
h_t     = 600;     % Final altitude [m]
rt = (h_t+Mars_radius);
<<<<<<< HEAD
e_t = Mars_mu / rt - 0.5 * V_t^2;
%% Parachute deployment constraints
% Targeting accuracy (within 5 km of target on the surface)
Rgo_max = 5e3;        % [m] max allowed range-to-go at deploy
=======
%% Targeting Accuracy Requirement
Rgo_target = 5e3;  % Maximum allowable targeting error [m] (5 km)
>>>>>>> parent of a4c5db4 (LNPCG implementation)

% Mach window for safe deployment (placeholder values – tune as needed)
M_min   = 1.5;        % [-] minimum Mach for chute deployment
M_max   = 2.5;        % [-] maximum Mach for chute deployment

% Dynamic pressure limit at deploy
q_chute_min = 3e2;           % [Pa] example value, tune from parachute spec
q_chute_max   = 8e2;        % [Pa] example value, tune from parachute spec

% Minimum altitude (relative to MOLA) to allow deployment
h_chute_min   = 6e3;      % [m] e.g. 0 km MOLA; increase if you want extra margin

% Bank-angle sanity check (optional for supervisor later)
sigma_max_allowed = 120*deg2rad;  % [rad] example max |sigma| 90º deegres with 10% margin to avoid saturation
sigma_max_rate = 20; %deg/s
sigma_max_rate_accel = 5; %deg/s^2

%% Logistic work
%Logistic Variables
sigma_logistic_0 = -120*deg2rad;
%K_logistic = 1.578;
%K_logistic = 0.944;
K_logistic = 2.89;
%% Entry Vehicle Aerodynamic Properties
mass = 2804; % Mass [kg]
S_ref = 15.9; % Reference area m^2
Cd = 1.45;
Cl = 0.36;
L_over_D = 0.2483;   % Lift-to-drag ratio, dimensionless
beta      = mass/(S_ref*Cd);   % Ballistic coefficient [kg/m^2]

%% Path Constraints and Requirements
A_path_max  = 15;  % Maximum allowable acceleration [m/s^2] (4g)
q_path_max  = 13e3;         % Maximum allowable dynamic pressure [Pa] (13 kPa)
Qdot_path_max = 500e3;      % Maximum allowable heat rate [W/m^2] (500 kW/m^2)

%% Compute Initial Range-to-Go

% Convert degrees to radians
deg2rad = pi/180;
lat0   = -21.3 * deg2rad;
lon0   = -176.40167 * deg2rad;
%lat_t  = 0.276 * deg2rad;
%lon_t  = -175.8 * deg2rad;

% Use the spherical_metrics function
%[d, sin_d, cos_d, RC, RD, RD_go, Rgo] = spherical_metrics(lat0, lon0, lat0, lon0, lat_t, lon_t, Mars_radius);

% Range-to-go is simply great-circle distance to target (d)
<<<<<<< HEAD
%fprintf('Initial Range-to-Go: %.2f km\n', Rgo/1000);
=======
fprintf('Initial Range-to-Go: %.2f km\n', Rgo/1000);
<<<<<<< HEAD

%% Initial Bank Angle Guess and Final Bank Angle
sigma0_guess_deg = 100;        % Initial guess for bank angle [deg]
sigma_f_deg      = 10;          % Final bank angle at target [deg]

% Convert to radians for MATLAB/Simulink
sigma0_guess = sigma0_guess_deg * deg2rad;
sigma_f      = sigma_f_deg * deg2rad;

fprintf('Initial Bank Guess: %.2f deg, Final Bank: %.2f deg\n', ...
    sigma0_guess_deg, sigma_f_deg);
>>>>>>> parent of a3f172f (Finally implemented LNPCG)
=======
>>>>>>> parent of a4c5db4 (LNPCG implementation)
