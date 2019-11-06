function [homRobot, homTarget] = homotheticTransformation(lambda, robot_coordinates,...
                                                        TargetAll, center, num)
% гомотетия 
x_1_sd = robot_coordinates(:,1);
y_1_sd = robot_coordinates(:,2);
z_1_sd = robot_coordinates(:,3);
Target_x = TargetAll(:,1);
Target_y = TargetAll(:,2);
Target_z = TargetAll(:,3);
x_c2 = center(:,1);
y_c2 = center(:,2);
z_c2 = center(:,3);

new_x_1_sd = zeros(1, num);
new_y_1_sd = zeros(1, num);
new_z_1_sd = zeros(1, num);

new_x_2 = zeros(1, size(Target_x,1));
new_y_2 = zeros(1, size(Target_x,1));
new_z_2 = zeros(1, size(Target_x,1));
for i = 1:num
       new_x_1_sd(i) = x_c2 + lambda*(x_1_sd(i) - x_c2);
       new_y_1_sd(i) = y_c2 + lambda*(y_1_sd(i) - y_c2); 
       new_z_1_sd(i) = z_c2 + lambda*(z_1_sd(i) - z_c2);
end
for i = 1:size(TargetAll,1)
       new_x_2(i) = x_c2 + lambda*(Target_x(i) - x_c2);
       new_y_2(i) = y_c2 + lambda*(Target_y(i) - y_c2);
       new_z_2(i) = z_c2 + lambda*(Target_z(i) - z_c2);
end

homRobot = [new_x_1_sd; new_y_1_sd; new_z_1_sd]';
homTarget = [new_x_2; new_y_2; new_z_2]';
