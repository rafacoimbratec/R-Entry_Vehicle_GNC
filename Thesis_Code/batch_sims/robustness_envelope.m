function [success_map, deploy_maps] = robustness_envelope(modelName)
    % Inputs:
    % modelName - the Simulink model name
    modelName = 'run_sim';

    % Constants
    deg2rad = pi/180;
    Mars_radius = 3396.2e3;      % [m]
    Mars_mu = 4.282837e13;       % [m^3/s^2] Mars GM
    rho0_nom = 0.020;            % [kg/m^3]
    mass_nom = 2804;             % [kg]
    L_over_D_nom = 0.2483;       % Lift-to-drag ratio
    A_path_max = 15;             % [m/s^2]
    q_path_max = 13e3;           % [Pa]
    Qdot_path_max = 500e3;       % [W/m^2]
    %Rgo_max = 5e3;               % [m]
    h_chute_min = 6e3;           % [m]
    M_min = 1.5;
    M_max = 2.5;

    % Define dispersion ranges
    N_gamma = 4;                 % Number of EFPA samples
    N_rhoscale = 4;              % Number of density-scale samples
    N_mass = 1;                  % Number of mass variations
    N_heading = 1;               % Number of heading variations
    N_LD = 1;                    % Number of L/D variations
    
    % Define EFPA, density scale, mass, heading, and L/D variations
    dgamma_deg = linspace(-0.3, 0.3, N_gamma);  % EFPA ±0.3°
    rho_scale_vec = linspace(0.95, 1.05, N_rhoscale);  % ±20% density scale
    mass_scale = linspace(0.95, 1.05, N_mass);       % ±5% mass
    heading_scale = linspace(-0.5, 0.5, N_heading);  % ±0.5° heading
    LD_scale = linspace(0.2383, 0.2583, N_LD);      % ±0.01 L/D

    gamma0_nom = -15;            % Nominal EFPA in degrees
    psi0_nom = -2.8758;          % Nominal heading in degrees

    % Initialize results matrices
    success_map = false(N_rhoscale, N_gamma, N_mass, N_heading, N_LD);  % deploy success
    deploy_maps = struct('h_deploy', [], 'Rgo_deploy', [], 'q_max_map', [], 'g_max_map', [], 'Qdot_max_map', []);

    % Loop through all the possible combinations of dispersions
    for i_rho = 1:N_rhoscale
        rho_scale_factor = rho_scale_vec(i_rho);
        for j_g = 1:N_gamma
            gamma0_deg = gamma0_nom + dgamma_deg(j_g);  % Perturbed EFPA
            gamma0 = gamma0_deg * deg2rad;

            for k_m = 1:N_mass
                mass_factor = mass_scale(k_m);  % Perturbed mass
                for l_h = 1:N_heading
                    heading_factor = heading_scale(l_h);  % Perturbed heading
                    for m_LD = 1:N_LD
                        LD_factor = LD_scale(m_LD);  % Perturbed L/D

                        % Assign values to base workspace for Simulink
                        assignin('base', 'rho_0', rho0_nom * rho_scale_factor);
                        assignin('base', 'gamma0', gamma0);
                        assignin('base', 'psi0', psi0_nom) %+ heading_factor);  % heading dispersion
                        assignin('base', 'mass', mass_nom) %* mass_factor);  % mass dispersion
                        assignin('base', 'L_over_D', L_over_D_nom) %* LD_factor);  % L/D dispersion

                        fprintf('Running simulation: EFPA = %.2f°, rho_scale = %.4f, mass = %.2f, heading = %.2f°, L/D = %.3f\n', ...
                                gamma0_deg, rho0_nom * rho_scale_factor, mass_nom, psi0_nom + 0, L_over_D_nom);

                        % Run Simulink simulation
                        simOut = sim(modelName, 'SaveOutput', 'on', 'SaveTime', 'on');

                        % Extract logs
                        t = simOut.tout;
                        state_ws = simOut.state_workspace;  % [r, theta, phi, V, gamma, psi, sigma]
                        spherical = simOut.spherical_metrics;  % [d, sin_d, cos_d, RC, RD, RD_go, Rgo]
                        path_ws = simOut.path_constraints;  % [gload, dynamic_pressure, Qdot]
                        mach_ts = simOut.mach;  % Mach time series
                        %deploy_flag = simOut.deployed_flag;  % final deploy flag (boolean time series)

                        % Unpack state
                        x_data = state_ws.signals.values;
                        r = x_data(:,1);
                        sph = spherical.signals.values;
                        d = sph(:,1);
                        RC = sph(:,4);
                        RD = sph(:,5);
                        Rgo = sph(:,7);

                        % Path constraint maxima
                        pc_data = path_ws.signals.values;
                        gload = pc_data(:,1);
                        q = pc_data(:,2);
                        Qdot = pc_data(:,3);
                        mach = mach_ts.signals.values;

                        % Determine if parachute deployed
                        % Determine if parachute deployed (altitude > 6 km)
                        h_dep = r(end) - Mars_radius;
                        deployed_final = h_dep >= h_chute_min;  % Parachute deployed if altitude > 6 km
                        %deployed_final = logical(deploy_flag.signals.values(end));

                        % Maxima for path constraints
                        g_max = max(gload);
                        q_max = max(q);
                        Qdot_max = max(Qdot);

                        violated = (g_max > A_path_max) || (q_max > q_path_max) || (Qdot_max > Qdot_path_max);

                        % Save maxima to result maps
                        deploy_maps.q_max_map(i_rho,j_g,k_m,l_h,m_LD) = q_max;
                        deploy_maps.g_max_map(i_rho,j_g,k_m,l_h,m_LD) = g_max;
                        deploy_maps.Qdot_max_map(i_rho,j_g,k_m,l_h,m_LD) = Qdot_max;

                        % If deployed successfully, record final state
                        if deployed_final
                            r_dep = r(end);
                            h_dep = r_dep - Mars_radius;
                            Mach_dep = mach(end);
                            Rgo_dep = Rgo(end);

                            deploy_maps.h_deploy(i_rho,j_g,k_m,l_h,m_LD) = h_dep;
                            deploy_maps.Rgo_deploy(i_rho,j_g,k_m,l_h,m_LD) = Rgo_dep;

                            % Deployment success criteria
                            success_map(i_rho,j_g,k_m,l_h,m_LD) = (h_dep >= h_chute_min) && ...
                                                                    (Mach_dep >= M_min) && (Mach_dep <= M_max) && ...
                                                                    ~violated;
                        end
                    end
                end
            end
        end
    end

    % Now that the simulation is completed for all dispersions, plot the design maps
    plot_design_maps(deploy_maps, success_map, gamma0_deg, rho_scale_vec, mass_scale, heading_scale, LD_scale, N_gamma,gamma0_nom);
end

function plot_design_maps(deploy_maps, success_map, gamma0_deg, rho_scale_vec, mass_scale, heading_scale, LD_scale,N_gamma,gamma0_nom)
    % Ensure gamma0_deg and rho_scale_vec are sorted and have no duplicates
    dgamma_deg = linspace(-1, 1, N_gamma);
    gamma0_deg = gamma0_nom + dgamma_deg;
    gamma0_deg = sort(unique(gamma0_deg));  % Sorting and removing duplicates
    rho_scale_vec = sort(unique(rho_scale_vec));  % Sorting and removing duplicates

    % Print and check the vectors
    disp('gamma0_deg values:');
    disp(gamma0_deg);
    disp('rho_scale_vec values:');
    disp(rho_scale_vec);

    % Check if the values are strictly increasing
    if any(diff(gamma0_deg) <= 0)
        error('gamma0_deg is not strictly increasing');
    end
    if any(diff(rho_scale_vec) <= 0)
        error('rho_scale_vec is not strictly increasing');
    end

    % Create the meshgrid for plotting
    [GammaGrid, RhoGrid] = meshgrid(gamma0_deg, rho_scale_vec);  % Create a 2D grid for gamma0_deg and rho_scale_vec

    % 1) Deploy success map
    figure;
    imagesc(gamma0_deg, rho_scale_vec, success_map(:,:,1,1,1)); % Example plot for success
    set(gca, 'YDir', 'normal');
    xlabel('Entry FPA [deg]');
    ylabel('Density scale factor \rho_{scale}');
    title('Deploy Success Map');
    colorbar;
    colormap(gray);

    % 2) Deploy altitude (contours)
    figure;
    contourf(GammaGrid, RhoGrid, deploy_maps.h_deploy(:,:,1,1,1)*10^-3, 20, 'LineColor', 'none');
    colorbar;
    xlabel('Entry FPA [deg]');
    ylabel('Density scale factor \rho_{scale}');
    title('Deploy Altitude [km]');
    grid on;

    % 3) Range-to-go at deployment (contours)
    figure;
    contourf(GammaGrid, RhoGrid, deploy_maps.Rgo_deploy(:,:,1,1,1)*10^-3, 20, 'LineColor', 'none');
    colorbar;
    xlabel('Entry FPA [deg]');
    ylabel('Density scale factor \rho_{scale}');
    title('Range-to-Go at Deployment [km]');
    grid on;

    % 4) Max dynamic pressure (contours)
    %figure;
    %contourf(GammaGrid, RhoGrid, deploy_maps.q_max_map(:,:,1,1,1), 20, 'LineColor', 'none');
    %hold on;
    %contour(GammaGrid, RhoGrid, deploy_maps.q_max_map(:,:,1,1,1), [q_path_max/1e3 q_path_max/1e3], 'k--', 'LineWidth', 1.5);
    %colorbar;
    %xlabel('Entry FPA [deg]');
    %ylabel('Density scale factor \rho_{scale}');
    %title('Max Dynamic Pressure [kPa]');
    %grid on;

    % 5) Max acceleration (contours)
    %figure;
    %contourf(GammaGrid, RhoGrid, deploy_maps.g_max_map(:,:,1,1,1), 20, 'LineColor', 'none');
    %hold on;
    %contour(GammaGrid, RhoGrid, deploy_maps.g_max_map(:,:,1,1,1), [A_path_max/9.80665 A_path_max/9.80665], 'k--', 'LineWidth', 1.5);
    %colorbar;
    %xlabel('Entry FPA [deg]');
    %ylabel('Density scale factor \rho_{scale}');
    %title('Max Acceleration [g]');
    %grid on;

    % 6) Max heat rate (contours)
    %figure;
    %contourf(GammaGrid, RhoGrid, deploy_maps.Qdot_max_map(:,:,1,1,1), 20, 'LineColor', 'none');
    %hold on;
    %contour(GammaGrid, RhoGrid, deploy_maps.Qdot_max_map(:,:,1,1,1), [Qdot_path_max/1e3 Qdot_path_max/1e3], 'k--', 'LineWidth', 1.5);
    %colorbar;
    %xlabel('Entry FPA [deg]');
    %ylabel('Density scale factor \rho_{scale}');
    %title('Max Heat Rate [kW/m^2]');
    %grid on;

    % 7) Plot for any other additional variables can follow a similar pattern
end



