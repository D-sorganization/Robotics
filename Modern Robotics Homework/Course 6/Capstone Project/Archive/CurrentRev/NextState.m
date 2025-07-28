function config_new = NextState(config, speeds, dt, max_speed)
% NextState - Simulates the kinematics of the youBot
% Inputs:
%   config - 12x1 vector: [phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4]
%            phi, x, y: chassis configuration (rad, m, m)
%            J1-J5: arm joint angles (rad)
%            W1-W4: wheel angles (rad)
%   speeds - 9x1 vector: [u1, u2, u3, u4, thetadot1-5]
%            u1-u4: wheel angular velocities (rad/s)
%            thetadot1-5: arm joint angular velocities (rad/s)
%   dt - timestep (s)
%   max_speed - maximum angular speed for wheels and joints (rad/s)
%
% Output:
%   config_new - 12x1 updated configuration vector

% Ensure column vectors
config = config(:);
speeds = speeds(:);

% Apply speed limits
speeds(speeds > max_speed) = max_speed;
speeds(speeds < -max_speed) = -max_speed;

% Extract current configuration
phi = config(1);
x = config(2);
y = config(3);
arm_angles = config(4:8);
wheel_angles = config(9:12);

% Extract speeds
wheel_speeds = speeds(1:4);
arm_speeds = speeds(5:9);

% Update arm joint angles (simple Euler integration)
arm_angles_new = arm_angles + arm_speeds * dt;

% Update wheel angles (simple Euler integration)
wheel_angles_new = wheel_angles + wheel_speeds * dt;

% Mecanum wheel kinematics for chassis update
% Robot parameters
r = 0.0475;  % wheel radius (m)
l = 0.235;   % half-length of robot (m)
w = 0.15;    % half-width of robot (m)

% F matrix for mecanum wheels (relates wheel speeds to body twist)
F = (r/4) * [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w);
              1,        1,       1,        1;
             -1,        1,      -1,        1];

% Calculate body twist Vb = [wbz, vbx, vby]
Vb = F * wheel_speeds;

% Odometry update
wbz = Vb(1);  % angular velocity
vbx = Vb(2);  % x velocity in body frame
vby = Vb(3);  % y velocity in body frame

% Calculate change in configuration
if abs(wbz) < 1e-6  % If angular velocity is essentially zero
    dqb = [0; vbx; vby];
else
    dqb = [wbz;
           (vbx*sin(wbz*dt) + vby*(cos(wbz*dt) - 1))/wbz;
           (vby*sin(wbz*dt) + vbx*(1 - cos(wbz*dt)))/wbz];
end

% Transform to world frame
dq = [1, 0,         0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)] * dqb * dt;

% Update chassis configuration
phi_new = phi + dq(1);
x_new = x + dq(2);
y_new = y + dq(3);

% Wrap phi to [-pi, pi]
phi_new = atan2(sin(phi_new), cos(phi_new));

% Assemble new configuration
config_new = [phi_new; x_new; y_new; arm_angles_new; wheel_angles_new];

end