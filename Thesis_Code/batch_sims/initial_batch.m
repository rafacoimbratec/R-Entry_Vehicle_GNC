%% batch_sim.m
% Run entry simulation and plot results
% Assumes signals are logged via To Workspace blocks
% in "Structure With Time" format (time + values inside simOut)

%% === Run Simulink model ===
modelName = 'run_sim';   % <-- replace with your model name
simOut = sim(modelName);

%% === Extract State Vector ===
state = state_workspace.signals.values;
t     = state_workspace.time;

% state = [r, theta, phi, V, gamma, psi, sigma]
r      = state(:,1);           % radial distance [m]
theta  = rad2deg(state(:,2));  % longitude [deg]
phi    = rad2deg(state(:,3));  % latitude [deg]
V      = state(:,4);           % velocity [m/s]
gamma  = rad2deg(state(:,5));  % flight path angle [deg]
psi    = rad2deg(state(:,6));  % heading angle [deg]
sigma  = rad2deg(state(:,7));  % bank angle [deg]

%% === Extract other logged variables ===
rho   = rho.signals.values;
T     = T.signals.values;
L     = L.signals.values;
D     = D.signals.values;
g     = g.signals.values;
gload = gload.signals.values;
q     = dynamic_pressure.signals.values;
Qdot  = Qdot.signals.values;

%% === Process variables ===
% Altitude [km] from Mars radius
Mars_radius = 3396.2e3; % [m]
altitude = (r - Mars_radius)/1e3; % km

% Temperature in °C
T_C = T - 273.15;

% Convert g-load to "g" units (Mars surface gravity)
gload_g = gload ./ 3.71;

%% === Path Constraints and Requirements ===
A_max     = 4;       % [g] max allowable load factor
q_max     = 13e3;    % [Pa] max dynamic pressure
Qdot_max  = 500e3;   % [W/m^2] max heat rate

%% === FIGURE 1: State Variables ===
figure('Name','Entry States','Color','w');

subplot(3,3,1)
plot(t, altitude); grid on;
xlabel('Time [s]'); ylabel('Altitude [km]');

subplot(3,3,2)
plot(t, V/1e3); grid on;
xlabel('Time [s]'); ylabel('Velocity [km/s]');

subplot(3,3,3)
plot(t, gamma); grid on;
xlabel('Time [s]'); ylabel('Flight Path Angle [deg]');

subplot(3,3,4)
plot(t, psi); grid on;
xlabel('Time [s]'); ylabel('Heading [deg]');

subplot(3,3,5)
plot(t, sigma); grid on;
xlabel('Time [s]'); ylabel('Bank Angle [deg]');

subplot(3,3,6)
plot(t, theta); grid on;
xlabel('Time [s]'); ylabel('Longitude [deg]');

subplot(3,3,7)
plot(t, phi); grid on;
xlabel('Time [s]'); ylabel('Latitude [deg]');

sgtitle('Mars Entry States');

%% === FIGURE 2: Path Constraints ===
figure('Name','Path Constraints','Color','w');

subplot(3,2,1)
plot(t, rho); grid on;
xlabel('Time [s]'); ylabel('\rho [kg/m^3]');

subplot(3,2,2)
plot(t, T_C); grid on;
xlabel('Time [s]'); ylabel('T [°C]');

subplot(3,2,3)
plot(t, gload_g); hold on; yline(A_max,'r--','LineWidth',1.5);
grid on;
xlabel('Time [s]'); ylabel('Load Factor [g]');
legend('g-load','A_{max}','Location','best');

subplot(3,2,4)
plot(t, q/1e3); hold on; yline(q_max/1e3,'r--','LineWidth',1.5);
grid on;
xlabel('Time [s]'); ylabel('q [kPa]');
legend('Dynamic Pressure','q_{max}','Location','best');

subplot(3,2,5)
plot(t, Qdot/1e3); hold on; yline(Qdot_max/1e3,'r--','LineWidth',1.5);
grid on;
xlabel('Time [s]'); ylabel('Heat Rate [kW/m^2]');
legend('Qdot','Qdot_{max}','Location','best');

sgtitle('Mars Entry Path Constraints');

%% === FIGURE 3: Trajectory Geometry and Metrics ===
figure('Name','Trajectory and Spherical Metrics','Color','w');

% --- Compute spherical metrics ---
lat  = phi*deg2rad;    % latitude [rad]
lon  = theta*deg2rad;  % longitude [rad]

N = length(t);

d      = zeros(N,1);
sin_d  = zeros(N,1);
cos_d  = zeros(N,1);
RC     = zeros(N,1);
RD     = zeros(N,1);
RD_go  = zeros(N,1);
Rgo    = zeros(N,1);

for k = 1:N
    [d(k), sin_d(k), cos_d(k), RC(k), RD(k), RD_go(k), Rgo(k)] = ...
        spherical_metrics(phi0, theta0, lat(k), lon(k), phi_t, theta_t, Mars_radius);
end

% --- Plot sigma vs time ---
subplot(3,3,1)
plot(t, sigma,'b','LineWidth',1.2); hold on;
plot(t(end), sigma(end),'ro','MarkerFaceColor','r');
grid on;
xlabel('Time [s]'); ylabel('\sigma [deg]');
title('Bank Angle vs Time');
text(t(end), sigma(end), sprintf('  (%.1f, %.1f°)', t(end), sigma(end)));

% --- Altitude vs Velocity ---
subplot(3,3,2)
plot(V/1e3, altitude,'b','LineWidth',1.2); hold on;
plot(V(end)/1e3, altitude(end),'ro','MarkerFaceColor','r');
grid on;
xlabel('Velocity [km/s]'); ylabel('Altitude [km]');
title('Altitude vs Velocity');
text(V(end)/1e3, altitude(end), sprintf('  (%.2f, %.1f)', V(end)/1e3, altitude(end)));

% --- Flight Path Angle vs Time ---
subplot(3,3,3)
plot(t, gamma,'b','LineWidth',1.2); hold on;
plot(t(end), gamma(end),'ro','MarkerFaceColor','r');
grid on;
xlabel('Time [s]'); ylabel('\gamma [deg]');
title('Flight Path Angle vs Time');
text(t(end), gamma(end), sprintf('  (%.1f, %.2f°)', t(end), gamma(end)));

% --- Latitude vs Longitude ---
subplot(3,3,4)
plot(theta, phi,'b','LineWidth',1.2); hold on;
plot(theta(end), phi(end),'ro','MarkerFaceColor','r');
grid on;
xlabel('Longitude [deg]'); ylabel('Latitude [deg]');
title('Ground Track (\phi vs \theta)');
text(theta(end), phi(end), sprintf('  (%.2f, %.2f)', theta(end), phi(end)));

% --- RD_go vs time ---
subplot(3,3,5)
plot(t, RD_go/1e3,'b','LineWidth',1.2); hold on;
plot(t(end), RD_go(end)/1e3,'ro','MarkerFaceColor','r');
grid on;
xlabel('Time [s]'); ylabel('RD_{go} [km]');
title('Remaining Downrange vs Time');
text(t(end), RD_go(end)/1e3, sprintf('  (%.1f, %.1f)', t(end), RD_go(end)/1e3));

% --- RC vs time ---
subplot(3,3,6)
plot(t, RC/1e3,'b','LineWidth',1.2); hold on;
plot(t(end), RC(end)/1e3,'ro','MarkerFaceColor','r');
grid on;
xlabel('Time [s]'); ylabel('RC [km]');
title('Crossrange vs Time');
text(t(end), RC(end)/1e3, sprintf('  (%.1f, %.1f)', t(end), RC(end)/1e3));

% --- Rgo vs time ---
subplot(3,3,7)
plot(t, Rgo/1e3,'b','LineWidth',1.2); hold on;
plot(t(end), Rgo(end)/1e3,'ro','MarkerFaceColor','r');
grid on;
xlabel('Time [s]'); ylabel('R_{go} [km]');
title('Total Range-to-go vs Time');
text(t(end), Rgo(end)/1e3, sprintf('  (%.1f, %.1f)', t(end), Rgo(end)/1e3));

sgtitle('Mars Entry Trajectory and Range Metrics');



