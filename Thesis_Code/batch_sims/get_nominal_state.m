%% Run nominal reference trajectory and save everything needed
modelName = 'run_sim';   % your Simulink model name

% Tell the model to use the nominal bank profile
%use_nominal_profile = true;
%assignin('base','use_nominal_profile',use_nominal_profile);

% Planet parameter
mu     = 4.282837e13;     % [m^3/s^2] Mars GM (same as in your dynamics)
Rplanet = 3396.2e3;       % [m] Mars radius (keep consistent)

% Run simulation
simOut = sim(modelName, 'SaveOutput','on', 'SaveTime','on');

%% Extract logs (adapt names exactly to your To-Workspace blocks)

t            = simOut.tout;                % time vector

state_ws     = simOut.state_workspace;     % [r, theta, phi, V, gamma, psi, sigma]
spherical    = simOut.spherical_metrics;   % [d, sin_d, cos_d, RC, RD, RD_go, Rgo]
aero_ws      = simOut.aero_workspace;      % [L, D, rho, g_planet]
path_ws      = simOut.path_constraints;    % [gload, dynamic_pressure, Qdot]
mach_ts      = simOut.mach;                % Mach (if separate)

% --- Unpack state ---
x_data   = state_ws.signals.values;
r        = x_data(:,1);
theta    = x_data(:,2);
phi      = x_data(:,3);
V        = x_data(:,4);
gamma    = x_data(:,5);
psi      = x_data(:,6);
sigma    = x_data(:,7);   % if stored there

% Altitude and specific mechanical energy
h        = r - Rplanet;
E        = -0.5 .* V.^2 + mu ./ r;   % your energy definition

% --- Aero / gravity ---
aero_data = aero_ws.signals.values;
L        = aero_data(:,1);
D        = aero_data(:,2);
rho      = aero_data(:,3);
g_planet = aero_data(:,4);   % gravity along track

% --- Path constraints ---
pc_data   = path_ws.signals.values;
gload     = pc_data(:,1);    % total accel (probably in m/s^2 or g – check)
q         = pc_data(:,2);    % dynamic pressure
Qdot      = pc_data(:,3);    % heat rate

% --- Mach ---
mach      = mach_ts.signals.values;

% --- Geometry ---
sph       = spherical.signals.values;
d         = sph(:,1);
RC        = sph(:,4);
RD        = sph(:,5);
RD_go     = sph(:,6);
Rgo       = sph(:,7);

%% Pack everything into a struct

ref = struct();
ref.t       = t;

% State
ref.r       = r;
ref.theta   = theta;
ref.phi     = phi;
ref.h       = h;
ref.V       = V;
ref.gamma   = gamma;
ref.psi     = psi;
ref.sigma   = sigma;

% Energy & environment
ref.E       = E;
ref.rho     = rho;
ref.g_planet= g_planet;

% Aero & loads
ref.L       = L;
ref.D       = D;
ref.q       = q;
ref.Qdot    = Qdot;
ref.gload   = gload;
ref.mach    = mach;

% Geometry
ref.d       = d;
ref.RC      = RC;
ref.RD      = RD;
ref.RD_go   = RD_go;
ref.Rgo     = Rgo;

%% Save in dedicated folder

outFolder = fullfile(pwd, 'nominal_reference');
if ~exist(outFolder, 'dir')
    mkdir(outFolder);
end

save(fullfile(outFolder, 'nominal_ref.mat'), 'ref');

fprintf('Saved nominal reference to %s\n', fullfile(outFolder, 'nominal_ref.mat'));

%% ============================================================
%  PLOTS – Figure 1: everything vs time
%% ============================================================

t_s = ref.t;                    % [s]
t_min = (t_s - t_s(1))/60;      % time from entry in minutes for nicer x-axis

% Conversions
h_km      = ref.h / 1e3;             % [km]
gamma_deg = ref.gamma * 180/pi;      % [deg]
psi_deg   = ref.psi   * 180/pi;      % [deg]
sigma_deg = ref.sigma * 180/pi;      % [deg]
q_kPa     = ref.q     / 1e3;         % [kPa]
Qdot_kW   = ref.Qdot  / 1e3;         % [kW/m^2]
g0        = 9.80665;
if max(ref.gload) > 20  % heuristic: if it's in m/s^2 convert to g
    g_load_g = ref.gload / g0;
else
    g_load_g = ref.gload;           % already in g
end
RD_km     = ref.RD / 1e3;
RC_km     = ref.RC / 1e3;
Rgo_km    = ref.Rgo / 1e3;

figure('Name','Nominal trajectory vs time','Position',[100 100 1400 900]);
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

% 1) Altitude
nexttile;
plot(t_min, h_km, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('Altitude [km]');
title('Altitude vs Time');

% 2) Velocity
nexttile;
plot(t_min, ref.V, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('V [m/s]');
title('Velocity vs Time');

% 3) Mach
nexttile;
plot(t_min, ref.mach, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('Mach [-]');
title('Mach vs Time');

% 4) Flight path angle
nexttile;
plot(t_min, gamma_deg, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('\gamma [deg]');
title('Flight Path Angle vs Time');

% 5) Heading
nexttile;
plot(t_min, psi_deg, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('\psi [deg]');
title('Heading vs Time');

% 6) Bank
nexttile;
plot(t_min, sigma_deg, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('\sigma [deg]');
title('Bank vs Time');

% 7) Dynamic pressure
nexttile;
plot(t_min, q_kPa, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('q [kPa]');
title('Dynamic Pressure vs Time');

% 8) Heat rate
nexttile;
plot(t_min, Qdot_kW, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('Q\_dot [kW/m^2]');
title('Heat Rate vs Time');

% 9) g-load
nexttile;
plot(t_min, g_load_g, 'LineWidth',1.5);
grid on;
xlabel('Time since entry [min]');
ylabel('g-load [g]');
title('Total Acceleration vs Time');

%% ============================================================
%  PLOTS – Figure 2: everything vs specific energy
%% ============================================================

E_MJ = ref.E / 1e6;   % [MJ/kg] since 1 m^2/s^2 = 1 J/kg

figure('Name','Nominal trajectory vs specific energy','Position',[200 200 1400 900]);
tiledlayout(3,3,'TileSpacing','compact','Padding','compact');

% 1) Altitude vs energy
nexttile;
plot(E_MJ, h_km, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('Altitude [km]');
title('Altitude vs Energy');

% 2) Velocity vs energy
nexttile;
plot(E_MJ, ref.V, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('V [m/s]');
title('Velocity vs Energy');

% 3) Mach vs energy
nexttile;
plot(E_MJ, ref.mach, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('Mach [-]');
title('Mach vs Energy');

% 4) Flight path angle vs energy
nexttile;
plot(E_MJ, gamma_deg, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('\gamma [deg]');
title('Flight Path Angle vs Energy');

% 5) Heading vs energy
nexttile;
plot(E_MJ, psi_deg, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('\psi [deg]');
title('Heading vs Energy');

% 6) Bank vs energy
nexttile;
plot(E_MJ, sigma_deg, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('\sigma [deg]');
title('Bank vs Energy');

% 7) Dynamic pressure vs energy
nexttile;
plot(E_MJ, q_kPa, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('q [kPa]');
title('Dynamic Pressure vs Energy');

% 8) Heat rate vs energy
nexttile;
plot(E_MJ, Qdot_kW, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('Q\_dot [kW/m^2]');
title('Heat Rate vs Energy');

% 9) g-load vs energy
nexttile;
plot(E_MJ, g_load_g, 'LineWidth',1.5);
grid on;
xlabel('Specific energy E [MJ/kg]');
ylabel('g-load [g]');
title('Total Acceleration vs Energy');

