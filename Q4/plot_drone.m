figure;
plot(result_position(2,:), result_position(3,:), 'o')
xlabel('Y-axix');
ylabel('Z-axix');
grid on;
title('2D trajectory');
figure;
scatter3(result_position(1,:), result_position(2,:), result_position(3,:), 'filled');  % 使用 'filled' 参数填充点

xlabel('X-axix');
ylabel('Y-axix');
zlabel('Z-axix');

grid on;
title("Tajectory of the quadcopter")