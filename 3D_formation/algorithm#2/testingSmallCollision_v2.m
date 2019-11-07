function [CollidedRobotsNum, CollidedRobotsKor] = testingSmallCollision_v2(PathKor, PathTime)
% ������������ ������� SmallCollisionTest.
% ����: PathKor, PathTime - ���������� ������� � ����� ������� � ����
% ��������������.
% �����: CollidedRobotsNum - ������ ������� � ���������� �� ������ ���������, 
% CollidedRobotsKor - ���������� ������� � ���������� �� ������ ���������.
% �����������: CollidedRobotsNumTest - ������ ������� � ���������� �� ��������� ���������,
% CollidedRobotsKorTest - ���������� ������� � ���������� �� ���������
% ���������.


global MINR 

ColNumCount = 0; 
CollidedRobotsNum = zeros((fix(size(PathKor,3)/3)), 2);     % ������ ������� � ���������� �� ������ ���������
CollidedRobotsKor = cell((fix(size(PathKor,3)/3)), 2);      % ���������� ������� � ���������� �� ������ ���������

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
%             msg = fprintf('����: �������� ����! ���������� %.4f', dist);
%             disp(msg);              
%         else
%             msg = fprintf('����: �������� ���, ���������� %.4f', dist);
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
    