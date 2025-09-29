%% Define spherical metrics parameters as a vector
% Order: [lat0, lon0, lat_t, lon_t, Rplanet]
spherical_metrics_params = [lat0, lon0, lat_t, lon_t, Mars_radius];

%% Define LNPCG guidance parameters as a vector
% Order: [sigma0_guess, sigma_f, e_f, Rplanet, mu, beta, L_over_D, omega, alpha1, alpha2]
LNPCG_params = [sigma0_guess, sigma_f, e_t, Mars_radius, Mars_mu, beta, L_over_D, Mars_omega, alpha1, alpha2];
