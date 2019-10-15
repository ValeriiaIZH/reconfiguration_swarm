function [A, E, V] = plotGraf(Peak, Edge, robotCoordinates, TargetAll, ...
                   PathKor_2, CollidedRobotsNum)
% функция по построению графа коллизий
NumOfPoints_1 = size(robotCoordinates(:,1));
NumOfPoints_2 = size(TargetAll(:,1)); 
f1 = figure;
figure(f1);
plotSurface(Peak, Edge, TargetAll); 
plotTraj(robotCoordinates,PathKor_2);                      
title('Граф коллизий');
% n = zeros(size(CollidedRobotsNum));
% m = zeros(size(CollidedRobotsNum));
A = zeros (size(robotCoordinates,1),size(robotCoordinates,1));
for i=1:size(CollidedRobotsNum)
    n(i) = CollidedRobotsNum(i,1); 
    m(i) = CollidedRobotsNum(i,2);
end
E = CollidedRobotsNum;
for i = 1:NumOfPoints_1
    for j = 1:NumOfPoints_2
     for t = 1:size(CollidedRobotsNum)
        if i==m(t) && j==n(t)
            A(i,j) = 1;
        else
        end
     end
    end
end
% disp(A);
% ROBCOOR_X = zeros(NumOfPoints_1);
% ROBCOOR_Y = zeros(NumOfPoints_1);
for i = 1:size(PathKor_2,3)
    ROBCOOR_X(i) = PathKor_2(1,1,i);
    ROBCOOR_Y(i) = PathKor_2(1,2,i);
%     text( PathKor_2(1,1,i), ...
%           PathKor_2(1,2,i), ...
%           PathKor_2(1,3,i), ...
%           num2str(i));
    text(TargetAll(i,1),... 
         TargetAll(i,2),...
         num2str(i));
end
V = [ROBCOOR_X; ROBCOOR_Y]';
% gplot(A, ROBCOOR, 'b-');
G = digraph(A);
p = plot(G, 'XData', ROBCOOR_X,'YData',ROBCOOR_Y);
for i = 1:NumOfPoints_1
    highlight(p,i,'NodeColor','m');
end
hold on;
grid on;
end

