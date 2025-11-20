%% ==============================================================
%  Constant bank-angle sweep (sigma_input) + footprint contours
%  - RC_final vs RD_final
%  - lat_final vs lon_final
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
%   simOut.q_dot.time
%   simOut.q_dot.signals.values               -> heat-rate q_dot
%
%  q_dot_max is treated as Q_dot_max (heat-rate max)
% ==============================================================
%Run main

%% -----------------------------
%  Sweep setup
% ------------------------------
modelName = 'run_sim';   % <-- change if needed

% Bank angle sweep [deg]
sigma_deg_vec = linspace(-70, 70, 5);
Ns = numel(sigma_deg_vec);

%% -----------------------------
%  Preallocate result arrays
% ------------------------------
RC_final      = zeros(Ns,1);   % crossrange  [m]
RD_final      = zeros(Ns,1);   % downrange   [m]
Rgo_final     = zeros(Ns,1);   % remaining range [m] (optional)

lat_final     = zeros(Ns,1);   % [rad]
lon_final     = zeros(Ns,1);   % [rad]
h_final       = zeros(Ns,1);   % [m]

gload_max     = zeros(Ns,1);   % max total g-load
q_max         = zeros(Ns,1);   % max dynamic pressure      [Pa]
Q_dot_max     = zeros(Ns,1);   % max heat-rate q_dot       [units of your q_dot]

% Constraint bookkeeping
constraint_any_viol  = false(Ns,1);   % any constraint violated (boolean)
n_constraints_viol   = zeros(Ns,1);   % number of violated constraints

%% -----------------------------
%  Path constraint limits
%  (set to your deploy/path limits)
% ------------------------------
% Example placeholders – replace with your actual values / variables:
g_limit    = A_path_max;   % [m/s^2]
q_limit    = q_path_max;      % [Pa]
Qdot_limit = Qdot_path_max;       % [whatever units q_dot uses]

% Planet radius if needed for altitude
Rplanet = Mars_radius;    % [m] (Earth example, replace if needed)


%% ==============================================================
%  1) Parameter sweep
% ==============================================================

for k = 1:Ns

    % ---------------------------------
    %  Set constant bank input for this run
    % ---------------------------------
    sigma_input = sigma_deg_vec(k)*deg2rad;       
    assignin('base','sigma_input',sigma_input);       % send to base for Simulink

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
    RC_final(k)  = sph_vals(end,4);   % RC [m]
    RD_final(k)  = sph_vals(end,5);   % RD [m]
    Rgo_final(k) = sph_vals(end,7);   % Rgo [m] (if you care)

    r_final      = st_vals(end,1);    % radius [m]
    theta_final  = st_vals(end,2);    % longitude [rad]
    phi_final    = st_vals(end,3);    % latitude  [rad]

    lon_final(k) = theta_final;
    lat_final(k) = phi_final;
    h_final(k)   = r_final - Rplanet; % altitude [m]

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

    gload_max(k) = max(g_vec);
    q_max(k)     = max(q_vals);
    Q_dot_max(k) = max(qdot_vals);   % heat rate max

    % ---------------------------------
    %  Constraint violations for this trajectory
    % ---------------------------------
    viol = false(3,1);
    viol(1) = gload_max(k) > g_limit;
    viol(2) = q_max(k)     > q_limit;
    viol(3) = Q_dot_max(k) > Qdot_limit;

    n_constraints_viol(k) = sum(viol);
    constraint_any_viol(k)= any(viol);

end

%% ==============================================================
%  RC vs RD footprint contours (colored)
% ==============================================================

nGrid   = 150;   % bump this up for smoother shapes
RC_grid = linspace(min(RC_final), max(RC_final), nGrid);   % [m]
RD_grid = linspace(min(RD_final), max(RD_final), nGrid);   % [m]
[RDg, RCg] = meshgrid(RD_grid, RC_grid);

F_h    = scatteredInterpolant(RD_final, RC_final, h_final,      'natural', 'none');
F_g    = scatteredInterpolant(RD_final, RC_final, gload_max,    'natural', 'none');
F_q    = scatteredInterpolant(RD_final, RC_final, q_max,        'natural', 'none');
F_Qdot = scatteredInterpolant(RD_final, RC_final, Q_dot_max,    'natural', 'none');
F_nV   = scatteredInterpolant(RD_final, RC_final, ...
                              double(n_constraints_viol),       'nearest', 'none');

h_grid     = F_h(RDg, RCg);
g_grid     = F_g(RDg, RCg);
q_grid     = F_q(RDg, RCg);
Qdot_grid  = F_Qdot(RDg, RCg);
nV_grid    = F_nV(RDg, RCg);

RDg_km    = RDg  / 1e3;
RCg_km    = RCg  / 1e3;
h_grid_km = h_grid / 1e3;

% Choose nice levels (tweak as you like)
h_levels   = linspace(floor(min(h_grid_km(:))/1)*1, ceil(max(h_grid_km(:))/1)*1, 10);
g_levels   = linspace(min(g_grid(:))/9.81, max(g_grid(:))/9.81, 10);
q_levels   = linspace(min(q_grid(:))/1e3,  max(q_grid(:))/1e3,  10);
Qd_levels  = linspace(min(Qdot_grid(:)),   max(Qdot_grid(:)),   10);
nV_levels  = 0:max(n_constraints_viol);

footprint_contour_color(RDg_km, RCg_km, h_grid_km, h_levels, ...
    'Deployment altitude, km', 'Downrange, km', 'Crossrange, km');

footprint_contour_color(RDg_km, RCg_km, g_grid/9.81, g_levels, ...
    'Max g-load, g', 'Downrange, km', 'Crossrange, km');

footprint_contour_color(RDg_km, RCg_km, q_grid/1e3, q_levels, ...
    'Max dynamic pressure, kPa', 'Downrange, km', 'Crossrange, km');

footprint_contour_color(RDg_km, RCg_km, Qdot_grid, Qd_levels, ...
    'Max heat rate Q\_dot', 'Downrange, km', 'Crossrange, km');

footprint_contour_color(RDg_km, RCg_km, nV_grid, nV_levels, ...
    'Number of constraint violations', 'Downrange, km', 'Crossrange, km');

%% ==============================================================
%  Latitude vs Longitude footprint contours (colored)
% ==============================================================

lat_deg = rad2deg(lat_final);
lon_deg = rad2deg(lon_final);

lat_grid = linspace(min(lat_deg), max(lat_deg), nGrid);
lon_grid = linspace(min(lon_deg), max(lon_deg), nGrid);
[LONG, LATG] = meshgrid(lon_grid, lat_grid);

Fh_ll    = scatteredInterpolant(lon_deg, lat_deg, h_final,      'natural','none');
Fg_ll    = scatteredInterpolant(lon_deg, lat_deg, gload_max,    'natural','none');
Fq_ll    = scatteredInterpolant(lon_deg, lat_deg, q_max,        'natural','none');
FQd_ll   = scatteredInterpolant(lon_deg, lat_deg, Q_dot_max,    'natural','none');
FnV_ll   = scatteredInterpolant(lon_deg, lat_deg, ...
                                double(n_constraints_viol),     'nearest','none');

h_ll_grid    = Fh_ll(LONG, LATG);
g_ll_grid    = Fg_ll(LONG, LATG);
q_ll_grid    = Fq_ll(LONG, LATG);
Qdot_ll_grid = FQd_ll(LONG, LATG);
nV_ll_grid   = FnV_ll(LONG, LATG);

h_ll_grid_km = h_ll_grid/1e3;

h_levels_ll  = linspace(floor(min(h_ll_grid_km(:))/1)*1, ceil(max(h_ll_grid_km(:))/1)*1, 10);
g_levels_ll  = linspace(min(g_ll_grid(:))/9.81, max(g_ll_grid(:))/9.81, 10);
q_levels_ll  = linspace(min(q_ll_grid(:))/1e3,  max(q_ll_grid(:))/1e3,  10);
Qd_levels_ll = linspace(min(Qdot_ll_grid(:)),   max(Qdot_ll_grid(:)),   10);
nV_levels_ll = 0:max(n_constraints_viol);

footprint_contour_color(LONG, LATG, h_ll_grid_km, h_levels_ll, ...
    'Deployment altitude, km (lat–lon)', 'Longitude, deg', 'Latitude, deg');

footprint_contour_color(LONG, LATG, g_ll_grid/9.81, g_levels_ll, ...
    'Max g-load, g (lat–lon)', 'Longitude, deg', 'Latitude, deg');

footprint_contour_color(LONG, LATG, q_ll_grid/1e3, q_levels_ll, ...
    'Max dynamic pressure, kPa (lat–lon)', 'Longitude, deg', 'Latitude, deg');

footprint_contour_color(LONG, LATG, Qdot_ll_grid, Qd_levels_ll, ...
    'Max heat rate Q\_dot (lat–lon)', 'Longitude, deg', 'Latitude, deg');

footprint_contour_color(LONG, LATG, nV_ll_grid, nV_levels_ll, ...
    'Number of constraint violations (lat–lon)', 'Longitude, deg', 'Latitude, deg');



function footprint_contour_color(X, Y, Z, levels, ttl, xlab, ylab)
%FOOTPRINT_CONTOUR_COLOR  Filled contour with black isolines + labels.

    figure; hold on; box on; grid on;

    % Filled contours (colors)
    if isempty(levels)
        [C,hf] = contourf(X, Y, Z, 'LineStyle','none');
    else
        [C,hf] = contourf(X, Y, Z, levels, 'LineStyle','none');
    end
    colormap(parula);    % or turbo, jet, etc.
    colorbar;

    % Overlay black contour lines
    if isempty(levels)
        [C2,h2] = contour(X, Y, Z, 'k', 'LineWidth', 0.8);
    else
        [C2,h2] = contour(X, Y, Z, levels, 'k', 'LineWidth', 0.8);
    end
    clabel(C2, h2, 'FontSize', 8, 'Color', 'k');

    xlabel(xlab);
    ylabel(ylab);
    title(ttl);

    axis equal tight;
end
