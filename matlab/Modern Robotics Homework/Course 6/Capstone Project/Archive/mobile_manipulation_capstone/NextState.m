function next_config = NextState(config, speeds, dt, max_speed)
speeds = min(max(speeds, -max_speed), max_speed);
phi = config(1); x = config(2); y = config(3);
theta_list = config(4:8);
wheel_angles = config(9:12);
arm_speeds = speeds(1:5);
wheel_speeds = speeds(6:9);
theta_list = theta_list + arm_speeds * dt;
wheel_angles = wheel_angles + wheel_speeds * dt;
[~, F] = youBotKinematics();
Vb = F * wheel_speeds;
wbz = Vb(1); vbx = Vb(2); vby = Vb(3);
if abs(wbz) < 1e-5
    delta_qb = [0, vbx, vby]' * dt;
else
    delta_qb = [wbz*dt;
                (vbx*sin(wbz*dt) + vby*(cos(wbz*dt)-1))/wbz;
                (vby*sin(wbz*dt) + vbx*(1-cos(wbz*dt)))/wbz];
end
delta_phi = delta_qb(1);
delta_x = delta_qb(2);
delta_y = delta_qb(3);
phi = phi + delta_phi;
x = x + (cos(phi)*delta_x - sin(phi)*delta_y);
y = y + (sin(phi)*delta_x + cos(phi)*delta_y);
next_config = [phi; x; y; theta_list; wheel_angles];
end