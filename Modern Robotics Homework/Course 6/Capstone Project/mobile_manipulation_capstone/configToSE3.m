function T = configToSE3(chassis, Tb0, T0e)
phi = chassis(1);
x = chassis(2);
y = chassis(3);
Tsb = [cos(phi), -sin(phi), 0, x;
       sin(phi),  cos(phi), 0, y;
       0,         0,        1, 0.0963;
       0,         0,        0, 1];
T = Tsb * Tb0 * T0e;
end