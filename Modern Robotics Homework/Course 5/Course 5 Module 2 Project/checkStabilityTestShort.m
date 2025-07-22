% TESTASSEMBLYSTABILITY Streamlined script to test the checkAssemblyStability function.
% This script defines two scenarios: one designed to be unstable, and one designed
% to be clearly stable, to verify the function's behavior.

clear; clc; % Clear workspace and command window
close all; % Close any open figures

fprintf('--- Starting Assembly Stability Tests ---\n');

% --- Scenario 1: Inherently Unstable Assembly (Expected: FALSE) ---
% This configuration makes Body 1 prone to tipping due to COM position
% and a destabilizing interaction force.

fprintf('\n>> Running Scenario 1: Unstable Assembly (Expected: FALSE)\n');

% Define Bodies for Scenario 1
bodies_s1(1).mass = 2;
bodies_s1(1).com = [25; 35]; % Body 1: COM far left of its single support
bodies_s1(2).mass = 5;
bodies_s1(2).com = [66; 42]; % Body 2: COM left of its support

% Define Contacts for Scenario 1
contacts_s1(1) = struct('body1', 1, 'body2', 0, 'normal', [0;1], 'position', [60; 0]); % Body 1 ground contact (rightmost point)
contacts_s1(2) = struct('body1', 2, 'body2', 0, 'normal', [0;1], 'position', [72; 0]); % Body 2 ground contact (rightmost point)
% Contact C: Body 1 to Body 2. Normal [1;0] points INTO body1.
% This means Body 2 pushes Body 1 to the RIGHT, exacerbating Body 1's tipping.
contacts_s1(3) = struct('body1', 1, 'body2', 2, 'normal', [1;0], 'position', [60; 60]);

mu_s1 = [0.1, 0.5, 0.5]; % Low friction under Body 1 (contact 1)

% Run stability check for Scenario 1
isStable_s1 = checkAssemblyStability(bodies_s1, contacts_s1, mu_s1);
fprintf('   Scenario 1 Result: %s (Expected: FALSE)\n', string(isStable_s1));
if ~isStable_s1
    fprintf('   -> Correct! Assembly is unstable as designed.\n');
else
    fprintf('   -> UNEXPECTED! This assembly should be unstable. Review Scenario 1 setup.\n');
end

% --- Scenario 2: Clearly Stable Assembly (Expected: TRUE) ---
% This setup provides stable bases for both bodies and a neutral/stabilizing interaction.

fprintf('\n>> Running Scenario 2: Stable Assembly (Expected: TRUE)\n');

% Define Bodies for Scenario 2
bodies_s2(1).mass = 10;
% Body 1: COM placed between its two ground contacts [10,0] and [50,0]
bodies_s2(1).com = [30; 35];
bodies_s2(2).mass = 10;
% Body 2: COM placed on its contact point [60,60] to avoid tipping moments from horizontal contact forces.
bodies_s2(2).com = [60; 42]; 

% Define Contacts for Scenario 2
contacts_s2(1) = struct('body1', 1, 'body2', 0, 'normal', [0;1], 'position', [10; 0]); % Body 1 LEFT ground contact
contacts_s2(2) = struct('body1', 2, 'body2', 0, 'normal', [0;1], 'position', [72; 0]); % Body 2 RIGHT ground contact
% Contact C: Body 1 to Body 2. Normal [-1;0] points INTO body1.
% This means Body 2 pushes Body 1 to the LEFT (stabilizing).
contacts_s2(3) = struct('body1', 1, 'body2', 2, 'normal', [-1;0], 'position', [60; 60]);
contacts_s2(4) = struct('body1', 1, 'body2', 0, 'normal', [0;1], 'position', [50; 0]); % Body 1 RIGHT ground contact
contacts_s2(5) = struct('body1', 2, 'body2', 0, 'normal', [0;1], 'position', [55; 0]); % Body 2 LEFT ground contact

mu_s2 = [0.5, 0.5, 0.5, 0.5, 0.5]; % Higher friction coefficients for all 5 contacts

% Run stability check for Scenario 2
isStable_s2 = checkAssemblyStability(bodies_s2, contacts_s2, mu_s2);
fprintf('   Scenario 2 Result: %s (Expected: TRUE)\n', string(isStable_s2));
if isStable_s2
    fprintf('   -> Correct! Assembly is stable as designed.\n');
else
    fprintf('   -> UNEXPECTED! This assembly should be stable. Check linprog diagnostics in checkAssemblyStability.m for details.\n');
end

fprintf('\n--- All Assembly Stability Tests Complete ---\n');