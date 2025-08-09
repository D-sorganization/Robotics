function GenerateCSV(filename, config_list, gripper_list)
n_steps = size(config_list, 1);
output = zeros(n_steps, 13);
for i = 1:n_steps
    output(i,:) = [config_list(i,:), gripper_list(i)];
end
writematrix(output, filename);
end