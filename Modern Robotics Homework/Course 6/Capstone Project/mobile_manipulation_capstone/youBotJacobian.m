function Je = youBotJacobian(joint_angles)
joint_angles = joint_angles(:);  % Ensure it's a column vector
[~, F] = youBotKinematics();
Slist = youBotArmSlist();
M = youBotArmHome();
T0e = FKinBody(M, Slist, joint_angles);
J_arm = JacobianBody(Slist, joint_angles);
r = 0.0475;
l = 0.235;
w = 0.15;
F6 = [0 0 0 0;
      0 0 0 0;
      1 1 1 1;
      0 -1 0 1;
      1 0 -1 0;
      -1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w)] * (r/4);
Teb = TransInv(T0e);
Adj_Teb = Adjoint(Teb);
J_base = Adj_Teb * F6;
Je = [J_arm J_base];
end