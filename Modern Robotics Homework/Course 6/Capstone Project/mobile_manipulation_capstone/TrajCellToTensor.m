function T_out = TrajCellToTensor(cell_array)
N = length(cell_array);
T_out = zeros(4, 4, N);
for i = 1:N
    T_out(:,:,i) = cell_array{i};
end
end