function q_new = ChassisOdometry(q, omega, dt)
% ChassisOdometry computes the updated chassis configuration given the
% current configuration, wheel angular velocities, and timestep.
%
% Inputs:
%   q: Current chassis configuration [phi; x; y]
%   omega: Wheel angular velocities [omega1; omega2; omega3; omega4]
%   dt: Timestep duration
%
% Output:
%   q_new: Updated chassis configuration [phi; x; y]

% Constants
r = 0.0475;  % Wheel radius [m]
l = 0.235;   % Half-length of chassis [m]
w = 0.15;    % Half-width of chassis [m]

% Kinematic matrix F based on wheel configuration
F = (r/4) * [ -1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
              1, 1, 1, 1;
              -1, 1, -1, 1];

% Chassis velocity in the chassis frame (Vb = [wbz; vbx; vby])
Vb = F * omega;

% Change in configuration
phi = q(1);
vbx = Vb(2);
vby = Vb(3);
wbz = Vb(1);

if abs(wbz) < 1e-5
    d_qb = [0; vbx; vby] * dt;
else
    d_qb = [wbz;
            (vbx * sin(wbz*dt) + vby * (cos(wbz*dt) - 1)) / wbz;
            (vby * sin(wbz*dt) + vbx * (1 - cos(wbz*dt))) / wbz];
end

% Transform to global frame
R = [1, 0, 0;
     0, cos(phi), -sin(phi);
     0, sin(phi), cos(phi)];

q_new = q + R * d_qb;

end
