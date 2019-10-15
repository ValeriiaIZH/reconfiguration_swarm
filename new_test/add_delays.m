function add_delays(num_active_robots,TargetAll,RobotCoordinates,CollidedRobotsNum)
global Vel DELAY;
% выходные данные: 
Av_compTime = 0;
Av_numOfCollisions = 0;
Av_maxDelaySum = 0;
Av_averDelaySum = 0; 
NumOfActiveRobot = num_active_robots;   
I = 50;
    for j = 1:I
        % точки траектории
        TrajPt  = NaN(NumOfActiveRobot, 2, 2);
            % начальные и конечные
        TrajPt(:, :, 1)  = RobotCoordinates;
        TrajPt(:, :, 2) = TargetAll(1:NumOfActiveRobot, :);
        TrajPt = single(TrajPt);
        if find(isnan(TrajPt))
            disp('ќбнаружен случай, не подход€щий под услови€ назначени€ траектории!');
        end
        % отрезки пути
        PathSeg      = cat(1, TrajPt(:,:,1:2));

        % рассто€ние между точками траекторий
        Dist_List = zeros( NumOfActiveRobot, 4);
        for i = 1 : ( size( TrajPt, 3 ) - 1 )
            Dist_List(:, i) = distBtw2Points(TrajPt(:,:,i),...
                                                 TrajPt(:,:,i+1));
        end
        Dist_List  = single(Dist_List);
        % врем€ в пути от точки до точки
        ElapsedTime = Dist_List/Vel;
        % отрезки рассто€ни€ между точками траекторий 
        Dist_List =   cat(1, Dist_List(:,1), ...
                            Dist_List(:,2));
        % временные отрезки пути
        % массив точек по траектори€м роботов
        TimePt       = single(zeros( NumOfActiveRobot, 5 ));
        for i = 1 : ( size( TrajPt, 3) - 1)
            TimePt(:, i + 1)  = TimePt(:, i) + ElapsedTime(:, i);
        end
        % временные отрезки пути по возрастанию времени
        TimeSeg      = cat(1, TimePt(:, 1:2));
        % номера роботов, соответствующие отрезкам времени и пути
        RobotIndx    = uint16(repmat((1 : NumOfActiveRobot)', size(TimePt, 2)-1, 1));
        % номера отрезков пути дл€ каждого робота
        SegNum = repmat((1:size(TimePt, 2)-1), NumOfActiveRobot, 1);
        SegNum = uint16(reshape(SegNum, [], 1));
        if length(SegNum(1,:)) < 2
                SegNum = SegNum';
        end
        % проверка на коллизии
        tic
        CollidedSeg = 0;
        iteration = 1;
        numOfCol = 0;
        while ~isempty(CollidedSeg)
            % пересекающиес€ по времени отрезки пути
            IntersectedSeg = intersectingPaths( TimeSeg, RobotIndx );
        %     timeInters = toc
            ColRobSort = sort(CollidedRobotsNum, 2);
            ColRobSort = unique(ColRobSort, 'rows');
            % поправка на задержки
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
        % выходные данные: 
        Av_compTime = Av_compTime + time;
        Av_numOfCollisions = Av_numOfCollisions + numOfCol;
        Av_maxDelaySum = Av_maxDelaySum + maxDelay;
        Av_averDelaySum = Av_averDelaySum + averageDelay;
    end
    % выходные данные: 
    AverCompTime = Av_compTime/I;
%     AverNumOfCollisions = Av_numOfCollisions/I;
    AverMaxDelaySum = Av_maxDelaySum/I;
%     AverDelay = Av_averDelaySum/I;
    disp('¬ыходные расчитанные параметры:');
%     msg = sprintf('—реднее врем€ вычислени€ = %6.4g', AverCompTime); 
%     disp(msg);
%     msg = sprintf('—реднее число коллизий = %6.4g', AverNumOfCollisions); 
%     disp(msg);
%     msg = sprintf('—редн€€ MAX задержка = %6.4g', AverMaxDelaySum); 
%     disp(msg);
%     msg = sprintf('—редн€€ обща€ задержка = %6.4g', AverDelay); 
%     disp(msg)
    msg = sprintf('ћаксимальна€ задержка = %d у робота є%d', maxDelay, iD);    
    disp(msg);
%     msg = sprintf('—редн€€ задержка = %d ', averageDelay);    
%     disp(msg);
end

