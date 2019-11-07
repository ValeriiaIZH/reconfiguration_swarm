function robot_coordinates = generate_robots(num, m, n)
x_1 = randi([m n],1,num);
y_1 = randi([m n],1,num);
robot_coordinates = [x_1; y_1]';
end

