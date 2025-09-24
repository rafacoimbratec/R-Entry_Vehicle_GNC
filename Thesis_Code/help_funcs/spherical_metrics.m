function [d, sin_d, cos_d, RC,RD, RD_go, Rgo] = spherical_metrics(lat0, lon0, lat, lon, lat_t, lon_t, Rplanet)
%SPHERICAL_METRICS Computes spherical distance, cross-range, and down-range metrics
%
% Inputs (all angles in radians):
%   lat0, lon0   - reference (entry) latitude and longitude
%   lat,  lon    - current latitude and longitude
%   lat_t, lon_t - target latitude and longitude
%   Rplanet      - planetary radius [m]
%
% Outputs:
%   d     - great-circle distance between reference and current point [m]
%   sin_d - sine of central angle
%   cos_d - cosine of central angle
%   RC    - cross-range distance [m]
%   RD    - down-range distance [m]

    % --- Central angle between reference (lat0, lon0) and current (lat, lon) ---
    cos_d = sin(lat0).*sin(lat) + cos(lat0).*cos(lat).*cos(lon-lon0);
    cos_d = min(max(cos_d,-1),1);         % clip to [-1,1] for acos
    d = acos(cos_d);                      % central angle [rad]
    sin_d = sqrt(max(0,1 - cos_d.^2));    % sine of central angle

    % --- Convert spherical (lat, lon) to Cartesian (unit sphere) ---
    rE = sph2cartvec(lat0, lon0);   % reference point
    rP = sph2cartvec(lat, lon);     % current point
    rT = sph2cartvec(lat_t, lon_t); % target point

    % --- Normals to trajectory and target planes ---
    nOET = cross(rE, rT); nOET = nOET / norm(nOET);
    nOEP = cross(rE, rP);
    if norm(nOEP) < 1e-12
        % Special case: entry point == current point
        % Use target plane normal directly to avoid NaN
        nOEP = nOET;  
    else
        nOEP = nOEP / norm(nOEP);
    end
    
    % --- Angle between trajectory plane and target plane ---
    i = acos(dot(nOEP, nOET));

    % --- Cross-range angle ---
    RC = asin(sin(i).*sin(d));

    % --- Sign of cross-range ---
    if dot(cross(nOEP,nOET)/norm(cross(nOEP,nOET)), rE) < 0
        RC = -RC;
    end

    % --- Down-range angle ---
    RD = acos(cos_d ./ cos(RC));

     % --- Total down-range (entry to target) ---
    cos_dt = sin(lat0).*sin(lat_t) + cos(lat0).*cos(lat_t).*cos(lon_t-lon0);
    cos_dt = min(max(cos_dt,-1),1);
    dt = acos(cos_dt);
    RD_tot = acos(cos_dt ./ cos(RC)); % along-track to target (rad)

    % --- Remaining downrange and total range-to-go ---
    RD_go = (RD_tot - RD) * Rplanet;
    Rgo   = sqrt( RD_go^2 + RC^2*Rplanet^2 );

    % --- Convert angular measures to distances ---
    RC = RC * Rplanet;
    RD = RD * Rplanet;
    d  = d  * Rplanet;

    % sin_d stays dimensionless
end

% --- Helper function: spherical to Cartesian (unit sphere) ---
function r = sph2cartvec(lat, lon)
    x = cos(lat) * cos(lon);
    y = cos(lat) * sin(lon);
    z = sin(lat);
    r = [x, y, z];
end



