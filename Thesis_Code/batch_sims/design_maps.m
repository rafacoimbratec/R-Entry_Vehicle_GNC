%% Design maps around nominal trajectory: EFPA vs density scale

clear; clc;

modelName = 'run_sim';   % <-- your Simulink model name

% -------------------------------------------------------------------------
% Load / define nominal parameters (must match your main script)
% -------------------------------------------------------------------------
deg2rad = pi/180;

Mars_radius = 3396.2e3;
Mars_mu     = 4.282837e13;

% Nominal entry conditions
gamma0_nom = -12 * deg2rad;
psi0_nom   = -2.8758 * deg2rad;

% Nominal atmosphere and vehicle
rho0_nom    = 0.020;        % [kg/m^3]
mass_nom    = 2804;         % [kg]
L_over_D_nom = 0.2483;

% Path/Deploy constraints (same as main script)
A_path_max     = 15;        % [m/s^2]
q_path_max     = 13e3;      % [Pa]
Qdot_path_max  = 500e3;     % [W/m^2]

Rgo_max        = 5e3;       % [m]
h_chute_min    = 6e3;       % [m]
M_min          = 1.5;
M_max          = 2.5;

% -------------------------------------------------------------------------
% Set model to use NOMINAL bank profile (open-loop reference)
% -------------------------------------------------------------------------
%use_nominal_profile = true;
%assignin('base','use_nominal_profile',use_nominal_profile);

% -------------------------------------------------------------------------
% Define dispersion grid: EFPA and density scale
% -------------------------------------------------------------------------
N_gamma   = 9;   % number of EFPA samples
N_rhoscale = 9;  % number of density-scale samples

dgamma_deg   = linspace(-1, 1, N_gamma);    % +/-1 deg around nominal
rho_scale_vec = linspace(0.8, 1.2, N_rhoscale); % +/-20% density scale

gamma0_deg_grid = -12 + dgamma_deg;         % for plotting
[GammaGrid, RhoGrid] = meshgrid(gamma0_deg_grid, rho_scale_vec);

% -------------------------------------------------------------------------
% Preallocate result arrays (rho_scale index = row, gamma index = col)
% -------------------------------------------------------------------------
h_deploy   = nan(N_rhoscale, N_gamma);    % deploy altitude [m]
Mach_deploy= nan(N_rhoscale, N_gamma);
Rgo_deploy = nan(N_rhoscale, N_gamma);    % [m]

q_max_map    = nan(N_rhoscale, N_gamma);  % [Pa]
g_max_map    = nan(N_rhoscale, N_gamma);  % [m/s^2]
Qdot_max_map = nan(N_rhoscale, N_gamma);  % [W/m^2]

success_map  = false(N_rhoscale, N_gamma); % true if successfully deployed
viol_map     = false(N_rhoscale, N_gamma); % true if any path constraint violated

% -------------------------------------------------------------------------
% Main sweep
% -------------------------------------------------------------------------
for i_rho = 1:N_rhoscale
    rho_scale_factor = rho_scale_vec(i_rho);
    %assignin('base','rho_scale_factor',rho_scale_factor);
    rho_0 = rho0_nom*rho_scale_factor;
    assignin('base','rho_0',rho_0);
    for j_g = 1:N_gamma
        gamma0_deg = gamma0_deg_grid(j_g);
        gamma0     = gamma0_deg * deg2rad;

        % Assign dispersed initial conditions to base workspace
        assignin('base','gamma0',gamma0);
        assignin('base','psi0',psi0_nom);      % heading nominal here
        assignin('base','mass',mass_nom);      % mass nominal here
        assignin('base','L_over_D',L_over_D_nom);

        fprintf('Running case: rho_scale = %.2f, gamma0 = %.2f deg\n', ...
                rho_scale_factor, gamma0_deg);

        % Run Simulink model
        simOut = sim(modelName, 'SaveOutput','on', 'SaveTime','on');

        % Extract logs
        t            = simOut.tout;
        state_ws     = simOut.state_workspace;   % [r, theta, phi, V, gamma, psi, sigma]
        spherical    = simOut.spherical_metrics; % [d, sin_d, cos_d, RC, RD, RD_go, Rgo]
        path_ws      = simOut.path_constraints;  % [gload, dynamic_pressure, Qdot]
        mach_ts      = simOut.mach;              % Mach time series
        deploy_flag  = simOut.deployed_flag;     % final deploy flag (boolean timeseries)

        % Unpack
        x_data   = state_ws.signals.values;
        r        = x_data(:,1);
        % theta    = x_data(:,2);
        % phi      = x_data(:,3);
        % V        = x_data(:,4);

        sph      = spherical.signals.values;
        d        = sph(:,1);
        RC       = sph(:,4);
        RD       = sph(:,5);
        RD_go    = sph(:,6);
        Rgo      = sph(:,7);

        pc_data  = path_ws.signals.values;
        gload    = pc_data(:,1);
        q        = pc_data(:,2);
        Qdot     = pc_data(:,3);

        mach     = mach_ts.signals.values;

        % Determine if parachute actually deployed (look at last sample)
        deployed_final = logical(deploy_flag.signals.values(end));

        % Path constraint maxima
        g_max   = max(gload);
        q_max   = max(q);
        Qdot_max= max(Qdot);

        violated = (g_max > A_path_max) || ...
                   (q_max > q_path_max) || ...
                   (Qdot_max > Qdot_path_max);

        % Save maxima in map
        q_max_map(i_rho,j_g)    = q_max;
        g_max_map(i_rho,j_g)    = g_max;
        Qdot_max_map(i_rho,j_g) = Qdot_max;
        viol_map(i_rho,j_g)     = violated;

        if deployed_final
            % Take last sample as deploy state
            r_dep    = r(end);
            h_dep    = r_dep - Mars_radius;
            Mach_dep = mach(end);
            Rgo_dep  = Rgo(end);

            h_deploy(i_rho,j_g)    = h_dep;
            Mach_deploy(i_rho,j_g) = Mach_dep;
            Rgo_deploy(i_rho,j_g)  = Rgo_dep;

            % Deploy success criteria (altitude, Rgo, Mach window)
            success = (h_dep >= h_chute_min) && ...
                      (Rgo_dep <= Rgo_max)   && ...
                      (Mach_dep >= M_min)    && ...
                      (Mach_dep <= M_max)    && ...
                      ~violated;

            success_map(i_rho,j_g) = success;
        else
            % No deploy: leave h_deploy/Mach_deploy/Rgo_deploy as NaN
            success_map(i_rho,j_g) = false;
        end

    end
end

%% Convert units for plotting
h_deploy_km   = h_deploy / 1e3;
Rgo_deploy_km = Rgo_deploy / 1e3;
q_max_kPa     = q_max_map / 1e3;
Qdot_max_kW   = Qdot_max_map / 1e3;
g_max_g       = g_max_map / 9.80665;   % if gload was m/s^2

%% 2D plots: EFPA vs density scale

% We'll use EFPA (deg) on x-axis, density scale on y-axis
[X, Y] = meshgrid(gamma0_deg_grid, rho_scale_vec);

% 1) Deploy success map
figure;
imagesc(gamma0_deg_grid, rho_scale_vec, success_map);
set(gca,'YDir','normal');
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('Deploy success map (1 = success, 0 = fail)');
colorbar;
colormap(gray);
grid on;

% 2) Deploy altitude
figure;
contourf(X, Y, h_deploy_km, 20, 'LineColor','none');
colorbar;
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('Deploy altitude [km]');
grid on;

% 3) Deploy range-to-go (targeting error)
figure;
contourf(X, Y, Rgo_deploy_km, 20, 'LineColor','none');
colorbar;
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('R_{go} at deploy [km]');
grid on;

% 4) Max dynamic pressure
figure;
contourf(X, Y, q_max_kPa, 20, 'LineColor','none');
hold on;
contour(X, Y, q_max_kPa, [q_path_max/1e3 q_path_max/1e3], 'k--', 'LineWidth',1.5);
colorbar;
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('Max dynamic pressure [kPa]');
grid on;

% 5) Max acceleration
figure;
contourf(X, Y, g_max_g, 20, 'LineColor','none');
hold on;
contour(X, Y, g_max_g, [A_path_max/9.80665 A_path_max/9.80665], 'k--', 'LineWidth',1.5);
colorbar;
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('Max acceleration [g]');
grid on;

% 6) Max heat rate
figure;
contourf(X, Y, Qdot_max_kW, 20, 'LineColor','none');
hold on;
contour(X, Y, Qdot_max_kW, [Qdot_path_max/1e3 Qdot_path_max/1e3], 'k--', 'LineWidth',1.5);
colorbar;
xlabel('Entry FPA [deg]');
ylabel('Density scale factor \rho_{scale}');
title('Max heat rate [kW/m^2]');
grid on;


