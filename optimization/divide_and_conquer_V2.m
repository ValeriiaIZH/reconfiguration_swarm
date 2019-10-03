function [S1, T1, S2, T2, numRob] = divide_and_conquer_V2(robot_coor,num_of_rob,...
                                                         targ_coor, num_of_tar)
R = sortrows(robot_coor);
T = R;
for i = 1:num_of_rob
    T(i,1) = R(num_of_rob+1-i,1);
    T(i,2) = R(num_of_rob+1-i,2);   
end
robot_coor = R;
clear T R;

R = sortrows(targ_coor);
T = R;
for i = 1:num_of_tar
    T(i,1) = R(num_of_tar+1-i,1);
    T(i,2) = R(num_of_tar+1-i,2);   
end
targ_coor = R;
clear T R;

if mod(num_of_rob,2) == 0
%     disp('количество роботов чётное');
    num_of_rob = num_of_rob/2;
    for i = 1:num_of_rob
        for j = 1:size(robot_coor,2)
            S1(i,j) = robot_coor(i,j);
            S2(i,j) = robot_coor(num_of_rob+i,j);
            T1(i,j) = targ_coor(i,j);
            T2(i,j) = targ_coor(num_of_rob+i,j);
        end
    end
    numRob = num_of_rob;
else
%     disp('количество роботов нечётное');
    num_of_rob_n = ceil(num_of_rob/2);
    for i = 1:num_of_rob_n
        for j = 1:size(robot_coor,2)
            S1(i,j) = robot_coor(i,j);
            T1(i,j) = targ_coor(i,j);
        end
    end
    rez = num_of_rob - num_of_rob_n;
    for i = 1:rez
        for j = 1:size(robot_coor,2)
            S2(i,j) = robot_coor(i+num_of_rob_n,j);
            T2(i,j) = targ_coor(i+num_of_rob_n,j);   
        end
    end
    numRob = num_of_rob_n;
end
end

