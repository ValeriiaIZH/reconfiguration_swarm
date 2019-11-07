function [CollidedRobotsNum, CollidedRobotsKor, distance] = my_collisiontest(PathKor, PathTime)
% функция на обнаружение коллизий между роботами
global MINR 

ColNumCount = 0; 
ColNumCount2 = 0;
CollidedRobotsNum = zeros((fix(size(PathKor,3)/3)), 2);     % номера роботов с коллизиями
CollidedRobotsKor = cell((fix(size(PathKor,3)/3)), 2);      % координаты роботов с коллизиями
IsCol = false;
for i = 1:(size(PathKor,3)-1) 
    StartPoint1 = PathKor(1,:, i);
    FinishPoint1 = PathKor(end,:, i);
%     disp(i);
    for j = (i+1):(size(PathKor,3))
        StartPoint2 = PathKor(1,:, j);
        FinishPoint2 = PathKor(end,:, j);
        
        L1 = [StartPoint1; FinishPoint1];
        L2 = [StartPoint2; FinishPoint2];
        
        [dist, ~, ~, ~] = DistBetween2Segment(StartPoint1, FinishPoint1, ...
                                                StartPoint2, FinishPoint2);  
        distance(i,j) = dist;                                    
        if dist < MINR
            IsCol = true;
            ColNumCount2 = ColNumCount2 +1; 
            CollidedRobotsNumTest(ColNumCount2, :) = [i, j];
            CollidedRobotsKorTest(ColNumCount2, :) = {L1, L2};
%             fprintf('Коллизия есть! Расстояние %.4f', dist);
%             fprintf('\n');
           
        else
            IsCol = false;
%             fprintf('Коллизии нет, расстояние %.4f', dist);
%             fprintf('\n');

        end
         if IsCol == true 
            ColNumCount = ColNumCount +1; 
            CollidedRobotsNum(ColNumCount, :) = [i, j];
            CollidedRobotsKor(ColNumCount, :) = {L1, L2};
        end
    end
end
CollidedRobotsNum((ColNumCount+1):end, :) = [];
CollidedRobotsKor((ColNumCount+1):end, :) = [];
