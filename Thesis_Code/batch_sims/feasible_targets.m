%% ==============================================================
%  Two-sigma bank strategy sweep + footprint contours
%  - RC_final vs RD_final
%  - lat_final vs lon_final
%
%  Simulink model must use:
%     sigma1_deg  (initial bank angle [deg])
%     sigma2_deg  (final bank angle [deg])
%  to build a linear schedule for sigma(t).
%
%  Assumes "Structure with time" logs:
%   simOut.spherical_metrics.time
%   simOut.spherical_metrics.signals.values   -> [ ..., RC, RD, ..., Rgo, ... ]
%
%   simOut.state_workspace.time
%   simOut.state_workspace.signals.values     -> [ r, theta, phi, v, gamma, psi, sigma ]
%
%   simOut.gload.time
%   simOut.gload.signals.values               -> accel (scalar or [gx gy gz])
%
%   simOut.dynamic_pressure.time
%   simOut.dynamic_pressure.signals.values    -> q
%
%   simOut.Qdot.time
%   simOut.Qdot.signals.values                -> heat-rate q_dot
%
%  q_dot_max is treated as Q_dot_max (heat-rate max)
% ==============================================================

% Run main (assumes A_path_max, q_path_max, Qdot_path_max, Mars_radius exist)
% main;   % <--- uncomment if you want to call your main script here

%% -----------------------------
%  Sweep setup
% ------------------------------
modelName = 'run_sim';   % Simulink model name

% Two-sigma sweep [deg]
sigma1_deg_vec = linspace(-81, 81, 2);   % initial sigma
sigma2_deg_vec = linspace(-81, 81, 2);   % final sigma

Ns1 = numel(sigma1_deg_vec);
Ns2 = numel(sigma2_deg_vec);

%% -----------------------------
%  Preallocate result arrays (Ns1 x Ns2)
% ------------------------------
RC_final      = zeros(Ns1,Ns2);   % crossrange  [m]
RD_final      = zeros(Ns1,Ns2);   % downrange   [m]
Rgo_final     = zeros(Ns1,Ns2);   % remaining range [m]

lat_final     = zeros(Ns1,Ns2);   % [rad]
lon_final     = zeros(Ns1,Ns2);   % [rad]
h_final       = zeros(Ns1,Ns2);   % [m]

gload_max     = zeros(Ns1,Ns2);   % max total g-load [m/s^2]
q_max         = zeros(Ns1,Ns2);   % max dynamic pressure [Pa]
Q_dot_max     = zeros(Ns1,Ns2);   % max heat-rate q_dot

% Constraint bookkeeping
constraint_any_viol  = false(Ns1,Ns2);     % any constraint violated
n_constraints_viol   = zeros(Ns1,Ns2);     % number of violated constraints

%% -----------------------------
%  Path constraint limits
% ------------------------------
g_limit    = A_path_max;      % [m/s^2]
q_limit    = q_path_max;      % [Pa]
Qdot_limit = Qdot_path_max;   % [units of Qdot]

% Planet radius for altitude
Rplanet = Mars_radius;        % [m]

%% ==============================================================
%  1) Two-sigma parameter sweep
% ==============================================================

for i = 1:Ns1
    for j = 1:Ns2

        sigma1_deg = sigma1_deg_vec(i);
        sigma2_deg = sigma2_deg_vec(j);

        % Send sigma parameters to base workspace for Simulink
        assignin('base','sigma1_deg',sigma1_deg);
        assignin('base','sigma2_deg',sigma2_deg);

        % ---------------------------------
        %  Run simulation
        % ---------------------------------
        simOut = sim(modelName, 'SaveOutput','on', 'SaveTime','on');

        % ---------------------------------
        %  Extract logs (Structure with time)
        % ---------------------------------
        spherical = simOut.spherical_metrics;
        state_ws  = simOut.state_workspace;
        g_str     = simOut.gload;
        q_str     = simOut.dynamic_pressure;
        qdot_str  = simOut.Qdot;          % heat-rate

        sph_vals  = spherical.signals.values;   % [N x n_sph]
        st_vals   = state_ws.signals.values;    % [N x n_state]
        g_vals    = g_str.signals.values;       % [N x 1] or [N x 3]
        q_vals    = q_str.signals.values;       % [N x 1]
        qdot_vals = qdot_str.signals.values;    % [N x 1]

        % ---------------------------------
        %  Final state quantities
        % ---------------------------------
        RC_final(i,j)  = sph_vals(end,4);   % RC [m]
        RD_final(i,j)  = sph_vals(end,5);   % RD [m]
        Rgo_final(i,j) = sph_vals(end,7);   % Rgo [m]

        r_final        = st_vals(end,1);    % radius [m]
        theta_final    = st_vals(end,2);    % longitude [rad]
        phi_final      = st_vals(end,3);    % latitude  [rad]

        lon_final(i,j) = theta_final;
        lat_final(i,j) = phi_final;
        h_final(i,j)   = r_final - Rplanet; % altitude [m]

        % ---------------------------------
        %  Path constraint maxima along trajectory
        % ---------------------------------
        % g-load magnitude
        if size(g_vals,2) > 1
            % If g is a vector [gx gy gz]
            g_vec = vecnorm(g_vals,2,2);
        else
            % Scalar g
            g_vec = g_vals;
        end

        gload_max(i,j) = max(g_vec);
        q_max(i,j)     = max(q_vals);
        Q_dot_max(i,j) = max(qdot_vals);   % heat rate max

        % ---------------------------------
        %  Constraint violations for this trajectory
        % ---------------------------------
        viol = false(3,1);
        viol(1) = gload_max(i,j) > g_limit;
        viol(2) = q_max(i,j)     > q_limit;
        viol(3) = Q_dot_max(i,j) > Qdot_limit;

        n_constraints_viol(i,j) = sum(viol);
        constraint_any_viol(i,j)= any(viol);

    end
end

%% ==============================================================
%  2) RC vs RD footprint contours (single figure with subplots)
% ==============================================================

nGrid   = 150;   % grid density for interpolation
RC_grid = linspace(min(RC_final(:)), max(RC_final(:)), nGrid);   % [m]
RD_grid = linspace(min(RD_final(:)), max(RD_final(:)), nGrid);   % [m]
[RDg, RCg] = meshgrid(RD_grid, RC_grid);

% Flatten data for scatteredInterpolant
RD_vec  = RD_final(:);
RC_vec  = RC_final(:);
h_vec   = h_final(:);
g_vecM  = gload_max(:);
q_vecM  = q_max(:);
Qd_vecM = Q_dot_max(:);
nV_vecM = double(n_constraints_viol(:));

F_h    = scatteredInterpolant(RD_vec, RC_vec, h_vec,      'natural', 'none');
F_g    = scatteredInterpolant(RD_vec, RC_vec, g_vecM,     'natural', 'none');
F_q    = scatteredInterpolant(RD_vec, RC_vec, q_vecM,     'natural', 'none');
F_Qdot = scatteredInterpolant(RD_vec, RC_vec, Qd_vecM,    'natural', 'none');
F_nV   = scatteredInterpolant(RD_vec, RC_vec, nV_vecM,    'nearest', 'none');

h_grid     = F_h(RDg, RCg);
g_grid     = F_g(RDg, RCg);
q_grid     = F_q(RDg, RCg);
Qdot_grid  = F_Qdot(RDg, RCg);
nV_grid    = F_nV(RDg, RCg);

RDg_km    = RDg  / 1e3;
RCg_km    = RCg  / 1e3;
h_grid_km = h_grid / 1e3;

% Levels (robust to NaNs / flat fields)
h_levels   = autoLevels(h_grid_km,   10);
g_levels   = autoLevels(g_grid/9.81, 10);
q_levels   = autoLevels(q_grid/1e3,  10);
Qd_levels  = autoLevels(Qdot_grid,   10);
nV_levels  = autoLevels(nV_grid,     max(n_constraints_viol(:))+1);

figure('Name','RC-RD footprint');
t = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% --- 1) Altitude ---
nexttile;
contourf(RDg_km, RCg_km, h_grid_km, h_levels, 'LineStyle','none');
hold on;
[C,hc] = contour(RDg_km, RCg_km, h_grid_km, h_levels, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Downrange, km'); ylabel('Crossrange, km');
title('Deployment altitude, km');
axis equal tight; grid on;

% --- 2) Max g ---
nexttile;
contourf(RDg_km, RCg_km, g_grid/9.81, g_levels, 'LineStyle','none');
hold on;
[C,hc] = contour(RDg_km, RCg_km, g_grid/9.81, g_levels, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Downrange, km'); ylabel('Crossrange, km');
title('Max g-load, g');
axis equal tight; grid on;

% --- 3) Max q ---
nexttile;
contourf(RDg_km, RCg_km, q_grid/1e3, q_levels, 'LineStyle','none');
hold on;
[C,hc] = contour(RDg_km, RCg_km, q_grid/1e3, q_levels, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Downrange, km'); ylabel('Crossrange, km');
title('Max dynamic pressure, kPa');
axis equal tight; grid on;

% --- 4) Max heat rate ---
nexttile;
contourf(RDg_km, RCg_km, Qdot_grid, Qd_levels, 'LineStyle','none');
hold on;
[C,hc] = contour(RDg_km, RCg_km, Qdot_grid, Qd_levels, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Downrange, km'); ylabel('Crossrange, km');
title('Max heat rate Q\_dot');
axis equal tight; grid on;

% --- 5) Number of violated constraints ---
nexttile;
contourf(RDg_km, RCg_km, nV_grid, nV_levels, 'LineStyle','none');
hold on;
[C,hc] = contour(RDg_km, RCg_km, nV_grid, nV_levels, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Downrange, km'); ylabel('Crossrange, km');
title('Constraint violations (count)');
axis equal tight; grid on;

title(t,'RC-RD footprint metrics');

%% ==============================================================
%  3) Latitude vs Longitude footprint contours (single figure)
% ==============================================================

lat_deg = rad2deg(lat_final(:));
lon_deg = rad2deg(lon_final(:));

lat_grid = linspace(min(lat_deg), max(lat_deg), nGrid);
lon_grid = linspace(min(lon_deg), max(lon_deg), nGrid);
[LONG, LATG] = meshgrid(lon_grid, lat_grid);

Fh_ll    = scatteredInterpolant(lon_deg, lat_deg, h_vec,      'natural','none');
Fg_ll    = scatteredInterpolant(lon_deg, lat_deg, g_vecM,     'natural','none');
Fq_ll    = scatteredInterpolant(lon_deg, lat_deg, q_vecM,     'natural','none');
FQd_ll   = scatteredInterpolant(lon_deg, lat_deg, Qd_vecM,    'natural','none');
FnV_ll   = scatteredInterpolant(lon_deg, lat_deg, nV_vecM,    'nearest','none');

h_ll_grid    = Fh_ll(LONG, LATG);
g_ll_grid    = Fg_ll(LONG, LATG);
q_ll_grid    = Fq_ll(LONG, LATG);
Qdot_ll_grid = FQd_ll(LONG, LATG);
nV_ll_grid   = FnV_ll(LONG, LATG);

h_ll_grid_km = h_ll_grid/1e3;

h_levels_ll  = autoLevels(h_ll_grid_km,   10);
g_levels_ll  = autoLevels(g_ll_grid/9.81, 10);
q_levels_ll  = autoLevels(q_ll_grid/1e3,  10);
Qd_levels_ll = autoLevels(Qdot_ll_grid,   10);
nV_levels_ll = autoLevels(nV_ll_grid,     max(n_constraints_viol(:))+1);

figure('Name','Lat-Lon footprint');
t2 = tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% --- 1) Altitude ---
nexttile;
contourf(LONG, LATG, h_ll_grid_km, h_levels_ll, 'LineStyle','none');
hold on;
[C,hc] = contour(LONG, LATG, h_ll_grid_km, h_levels_ll, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Longitude, deg'); ylabel('Latitude, deg');
title('Deployment altitude, km');
axis equal tight; grid on;

% --- 2) Max g ---
nexttile;
contourf(LONG, LATG, g_ll_grid/9.81, g_levels_ll, 'LineStyle','none');
hold on;
[C,hc] = contour(LONG, LATG, g_ll_grid/9.81, g_levels_ll, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Longitude, deg'); ylabel('Latitude, deg');
title('Max g-load, g');
axis equal tight; grid on;

% --- 3) Max q ---
nexttile;
contourf(LONG, LATG, q_ll_grid/1e3, q_levels_ll, 'LineStyle','none');
hold on;
[C,hc] = contour(LONG, LATG, q_ll_grid/1e3, q_levels_ll, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Longitude, deg'); ylabel('Latitude, deg');
title('Max dynamic pressure, kPa');
axis equal tight; grid on;

% --- 4) Max heat rate ---
nexttile;
contourf(LONG, LATG, Qdot_ll_grid, Qd_levels_ll, 'LineStyle','none');
hold on;
[C,hc] = contour(LONG, LATG, Qdot_ll_grid, Qd_levels_ll, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Longitude, deg'); ylabel('Latitude, deg');
title('Max heat rate Q\_dot');
axis equal tight; grid on;

% --- 5) Constraint violations ---
nexttile;
contourf(LONG, LATG, nV_ll_grid, nV_levels_ll, 'LineStyle','none');
hold on;
[C,hc] = contour(LONG, LATG, nV_ll_grid, nV_levels_ll, 'k','LineWidth',0.6);
clabel(C,hc,'FontSize',7);
colorbar;
xlabel('Longitude, deg'); ylabel('Latitude, deg');
title('Constraint violations (count)');
axis equal tight; grid on;

title(t2,'Lat-Lon footprint metrics');

%% ==============================================================
%  Helper: robust level generation
% ==============================================================

function levels = autoLevels(Z, nLevels)
%AUTLEVELS Create contour levels from data Z, robust to NaNs / empties.

    Zf = Z(:);
    Zf = Zf(isfinite(Zf));      % keep only finite values

    if numel(Zf) < 2
        % Not enough data to define a range -> let MATLAB choose
        levels = [];
        return;
    end

    zmin = min(Zf);
    zmax = max(Zf);

    if zmin == zmax
        % Completely flat field -> single contour level
        levels = zmin;
    else
        levels = linspace(zmin, zmax, nLevels);
    end
end

