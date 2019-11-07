function [CollidedRobotsNum, CollidedRobotsKor] = testingSmallCollision_v2(PathKor, PathTime)
% Тестирование функции SmallCollisionTest.
% Вход: PathKor, PathTime - координаты роботов и время роботов в пути
% соответственно.
% Выход: CollidedRobotsNum - номера роботов с коллизиями по нашему алгоритму, 
% CollidedRobotsKor - координаты роботов с коллизиями по нашему алгоритму.
% Опционально: CollidedRobotsNumTest - номера роботов с коллизиями по тестовому алгоритму,
% CollidedRobotsKorTest - координаты роботов с коллизиями по тестовому
% алгоритму.


global MINR 

ColNumCount = 0; 
CollidedRobotsNum = zeros((fix(size(PathKor,3)/3)), 2);     % номера роботов с коллизиями по нашему алгоритму
CollidedRobotsKor = cell((fix(size(PathKor,3)/3)), 2);      % координаты роботов с коллизиями по нашему алгоритму

for i = 1:(size(PathKor,3)-1) 
    StartPoint1 = PathKor(1,:, i);
    FinishPoint1 = PathKor(end,:, i);
%     disp(i);
    for j = (i+1):(size(PathKor,3))
        StartPoint2 = PathKor(1,:, j);
        FinishPoint2 = PathKor(end,:, j);
        
        L1 = [StartPoint1; FinishPoint1];
        L2 = [StartPoint2; FinishPoint2];
        
%         [dist, ~, Pc, Qc] = DistBetween2Segment(StartPoint1, FinishPoint1, ...
%                                                 StartPoint2, FinishPoint2);        
%         if dist < MINR
%             ColNumCount2 = ColNumCount2 +1; 
%             CollidedRobotsNum(ColNumCount2, :) = [i, j];
%             CollidedRobotsKor(ColNumCount2, :) = {L1, L2};
%             msg = fprintf('Тест: Коллизия есть! Расстояние %.4f', dist);
%             disp(msg);              
%         else
%             msg = fprintf('Тест: Коллизии нет, расстояние %.4f', dist);
%             disp(msg); 
%         end
        
        IsCol = SmallCollisionTest(0, min(PathTime(i), PathTime(j)),   ...
                                   StartPoint1, FinishPoint1, ...
                                   StartPoint2, FinishPoint2);
        
        if IsCol == true 
            ColNumCount = ColNumCount +1; 
            CollidedRobotsNum(ColNumCount, :) = [i, j];
            CollidedRobotsKor(ColNumCount, :) = {L1, L2};
        end
    end
end

CollidedRobotsNum((ColNumCount+1):end, :) = [];
CollidedRobotsKor((ColNumCount+1):end, :) = [];

% varargout(1) = {CollidedRobotsNumTest};      
% varargout(2) = {CollidedRobotsKorTest};   
    