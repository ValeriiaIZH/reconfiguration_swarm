function addDelays(CollidedRobotsNum, TargetCor, ActiveRobots)
% ������� ���������� ��������
global VEL TIMESTEP;
koef = 0.5;
DELAY = koef*TIMESTEP;
NumOfActiveRobot = size(ActiveRobots,1);

% �������� ������: 
Av_compTime = 0;
Av_numOfCollisions = 0;
Av_maxDelaySum = 0;
Av_averDelaySum = 0; 

% I = 50;
%     for j = 1:I
        % ����� ����������
        TrajPt  = NaN(NumOfActiveRobot, 3, 2);
            % ��������� � ��������
        TrajPt(:, :, 1)  = ActiveRobots;
        TrajPt(:, :, 2) = TargetCor(1:NumOfActiveRobot, :);
        TrajPt = single(TrajPt);
        if find(isnan(TrajPt))
            disp('��������� ������, �� ���������� ��� ������� ���������� ����������!');
        end
        % ������� ����
        PathSeg      = cat(1, TrajPt(:,:,1:2));

        % ���������� ����� ������� ����������
        Dist_List = zeros( NumOfActiveRobot, 4);
        for i = 1 : ( size( TrajPt, 3 ) - 1 )
            Dist_List(:, i) = distBtw2Points(TrajPt(:,:,i),...
                                                 TrajPt(:,:,i+1));
        end
        Dist_List  = single(Dist_List);
        % ����� � ���� �� ����� �� �����
        ElapsedTime = Dist_List/VEL;
        % ������� ���������� ����� ������� ���������� 
        Dist_List =   cat(1, Dist_List(:,1), ...
                            Dist_List(:,2));
        % ��������� ������� ����
        % ������ ����� �� ����������� �������
        TimePt       = single(zeros( NumOfActiveRobot, 5 ));
        for i = 1 : ( size( TrajPt, 3) - 1)
            TimePt(:, i + 1)  = TimePt(:, i) + ElapsedTime(:, i);
        end
        % ��������� ������� ���� �� ����������� �������
        TimeSeg      = cat(1, TimePt(:, 1:2));
        % ������ �������, ��������������� �������� ������� � ����
        RobotIndx    = uint16(repmat((1 : NumOfActiveRobot)', size(TimePt, 2)-1, 1));
        % ������ �������� ���� ��� ������� ������
        SegNum = repmat((1:size(TimePt, 2)-1), NumOfActiveRobot, 1);
        SegNum = uint16(reshape(SegNum, [], 1));
        if length(SegNum(1,:)) < 2
                SegNum = SegNum';
        end
        % �������� �� ��������
        tic
        CollidedSeg = 0;
        iteration = 1;
        numOfCol = 0;
        while ~isempty(CollidedSeg)
            % �������������� �� ������� ������� ����
            IntersectedSeg = intersectingPaths( TimeSeg, RobotIndx );
        %     timeInters = toc
            ColRobSort = sort(CollidedRobotsNum, 2);
            ColRobSort = unique(ColRobSort, 'rows');
            % �������� �� ��������
            TimePt(ColRobSort(:,2),:) = TimePt(ColRobSort(:,2),:) + DELAY;
            TimeSeg      = cat(1, TimePt(:, 1:2));
            iteration = iteration + 1;
            if iteration == 51
%                 disp('too many iterations!');
                break;
            end
        end
        time = toc;
        [maxDelay, iD] = max(TimePt(:,1));
        averageDelay = sum(TimePt(:,1))/NumOfActiveRobot;
%         % �������� ������: 
        Av_compTime = Av_compTime + time;
        Av_numOfCollisions = Av_numOfCollisions + numOfCol;
        Av_maxDelaySum = Av_maxDelaySum + maxDelay;
        Av_averDelaySum = Av_averDelaySum + averageDelay;
%     end
    % �������� ������: 
%     AverCompTime = Av_compTime/I;
%     AverNumOfCollisions = Av_numOfCollisions/I;
%     AverMaxDelaySum = Av_maxDelaySum/I;
%     AverDelay = Av_averDelaySum/I;
    disp('�������� ����������� ���������:');
%     msg = sprintf('������� ����� ���������� = %6.4g', AverCompTime); 
%     disp(msg);
%     msg = sprintf('������� ����� �������� = %6.4g', AverNumOfCollisions); 
%     disp(msg);
    msg = sprintf('����� �������� ����� ���������� �������� = %6.4g', Av_numOfCollisions); 
    disp(msg);
%     msg = sprintf('������� MAX �������� = %6.4g', AverMaxDelaySum); 
%     disp(msg);
%     msg = sprintf('������� ����� �������� = %6.4g', AverDelay); 
%     disp(msg)
%     msg = sprintf('������������ �������� = %d � ������ �%d', maxDelay, iD);    
%     disp(msg);
    msg = sprintf('������� �������� = %d ', averageDelay);    
    disp(msg);
end


