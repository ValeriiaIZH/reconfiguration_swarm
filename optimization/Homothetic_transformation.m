function [new_x_1_sd,new_y_1_sd, ...
          new_x_2, new_y_2] = Homothetic_transformation(lambda, robot_coordinates,...
                                                        TargetAll, center, num)
% гомотетия 
x_1_sd = robot_coordinates(:,1);
y_1_sd = robot_coordinates(:,2);
x_c2 = center(:,1);
y_c2 = center(:,2);

for i = 1:num

       new_x_1_sd(i) = x_c2 + max(lambda(:))*(x_1_sd(i)-x_c2);
       new_y_1_sd(i) = y_c2 + max(lambda(:))*(y_1_sd(i)-y_c2); 

       new_x_2(i) = x_c2 + max(lambda(:)).*(TargetAll(i,1)-x_c2);
       new_y_2(i) = y_c2 + max(lambda(:)).*(TargetAll(i,2)-y_c2);

end


