% тест раздел€й и властвуй
clc;
clearvars;
close all;
global Vel TIMESTEP;
Vel = 1;
TIMESTEP = 1;
num = input('num = ');
num_of_rob = num;
robot_coor = randi([0 20], num_of_rob, 2);

num_of_tar = num;
targ_coor = randi([10 15], num_of_tar, 2);

plot(robot_coor(:,1), robot_coor(:,2), 'or');
hold on;
grid on;
plot(targ_coor(:,1), targ_coor(:,2), 'ob');

[S1, T1, S2, T2] = divide_and_conquer_V2(robot_coor,num_of_rob,...
                                         targ_coor, num_of_tar);

[PathKor, PathTime] = calculate_rob_to_tar(num, targ_coor, robot_coor);
[CollidedRobotsNum,CollidedRobotsKor, dist] = my_collisiontest(PathKor, PathTime);

[PathKor1, PathTime1] = calculate_rob_to_tar_v1(num, T1, S1);
[CollidedRobotsNum1,CollidedRobotsKor1, dist1] = my_collisiontest(PathKor1, PathTime1);

[PathKor2, PathTime2] = calculate_rob_to_tar_v1(num, T2, S2);
[CollidedRobotsNum2,CollidedRobotsKor2, dist2] = my_collisiontest(PathKor2, PathTime2);

if isempty(CollidedRobotsNum1)
    disp('ok');
else
    [S1_1, T1_1, S1_2, T1_2] = divide_and_conquer_V2(S1, size(S1(:,1)),...
                                                     T1, size(T1(:,1)));
    [PathKor1_1, PathTime1_1] = calculate_rob_to_tar_v1(num, T1, S1);
    [CollidedRobotsNum1_1,CollidedRobotsKor1_1,...
                dist1_1] = my_collisiontest(PathKor1_1, PathTime1_1);
end

% disp('Collision between 2 robots');
% disp(CollidedRobotsNum);
% disp('Collision between rob at 1 part');
% disp(CollidedRobotsNum1);
% disp('Collision between rob at 2 part');
% disp(CollidedRobotsNum2);


