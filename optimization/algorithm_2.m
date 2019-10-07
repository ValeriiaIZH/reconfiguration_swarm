function [ActiveDist] = algorithm_2(Peak, Edge, num_of_robots, new_rob_cor, TargetAll,...
                     PathKor0,CollidedRobotsNum0)
    % построение графа коллизий
    % АЛГОРИТМ ПОСТРОЕНИЯ ГРАФА И НАХОЖДЕНИЯ В НЕМ ЦИКЛОВ (АЛГОРИТМ №3)
    
    [A, E, V] = plotGraf(Peak, Edge, new_rob_cor, TargetAll, ...
                  PathKor0, CollidedRobotsNum0);
%   [A, E, V] = plotGraf(Peak, Edge, new_rob_cor, TargetAll, ...
%                 ACTIVEBOBOTS, PathKor0, CollidedRobotsNum0);
    if max(A(:)) == 1
%     grPlot(V,E,'g','%d',''); % рисуем граф
%     set(get(gcf,'CurrentAxes'),...
%     'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%     title('\bfИсходный граф')
    Cycles = grCycleBasis(E); % все независимые циклы
%     for k1 = 1:size(Cycles,2)
%         grPlot(V,E(find(Cycles(:,k1)),:),'g','%d',''); % очередной цикл
%         set(get(gcf,'CurrentAxes'),...
%         'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%         title(['\bfЦикл N' num2str(k1)]);
%     end
    else 
       Cycles = []; 
    end
    if isempty(Cycles)
    disp('В графе коллизий нет циклов.');
    % назначение задержек
    add_delays(num_of_robots,TargetAll,new_rob_cor,CollidedRobotsNum0); % дописать функцию движения роботов с задержкой
    [~, ~, ActiveDist] = calculate_rob_to_tar(num_of_robots, TargetAll, new_rob_cor);
    clear A E V ;
    else
   disp('В графе коллизий имеются циклы! Алгоритм не работает.Применяем алгоритм "Разделяй и властвуй"');
    % АЛГОРИТМ РАЗДЕЛЯЙ И ВЛАСТВУЙ (АЛГОРИТМ №4)
    % уравнение прямой
    [S1, T1, S2, T2, numRob] = divide_and_conquer_V2(new_rob_cor, size(new_rob_cor,1), ...
                                             TargetAll, size(TargetAll,1)); %КОЛИЧЕСТВО ЦЕЛЕЙ!!!!!!!!!!!!!!!!!!!!!!!!!!
    [PathKor_S1, PathTime_S1, ActiveDist_S1] = calculate_rob_to_tar(numRob(1,1), T1, S1);
    % проверка на коллизии
    [CollidedRobotsNum_S1,...
            ~] = my_collisiontest(PathKor_S1, PathTime_S1);
    [A, E, V] = plotGraf(Peak, Edge, S1, T1, ...
                  PathKor_S1, CollidedRobotsNum_S1);
    if max(A(:)) == 1
%     figure
%     grPlot(V,E,'g','%d',''); % рисуем граф
%     set(get(gcf,'CurrentAxes'),...
%     'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%     title('\bfИсходный граф')
    Cycles = grCycleBasis(E); % все независимые циклы
%     for k1 = 1:size(Cycles,2)
%         grPlot(V,E(find(Cycles(:,k1)),:),'g','%d',''); % очередной цикл
%         set(get(gcf,'CurrentAxes'),...
%         'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%         title(['\bfЦикл N' num2str(k1)]);
%     end
    end
    if isempty(Cycles)
    disp('В графе коллизий нет циклов.');
    add_delays(numRob(1,1),T1,S1,CollidedRobotsNum_S1);
    else
%         disp('NEXT');
        ActiveDist_1 = algorithm_2(Peak, Edge, numRob(1,1), S1, T1,...
                                 PathKor_S1,CollidedRobotsNum_S1);
    end
    
    clear A E V ;
    [PathKor_S2, PathTime_S2, ActiveDist_S2] = calculate_rob_to_tar(numRob(1,2), T2, S2);
    ActiveDist = [ActiveDist_S1; ActiveDist_S2];
    % проверка на коллизии
    [CollidedRobotsNum_S2,...
            ~] = my_collisiontest(PathKor_S2, PathTime_S2);
    [A, E, V] = plotGraf(Peak, Edge, S2, T2, ...
                  PathKor_S2, CollidedRobotsNum_S2);
%     figure
    if max(A(:)) == 1
%     grPlot(V,E,'g','%d',''); % рисуем граф
%     set(get(gcf,'CurrentAxes'),...
%     'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%     title('\bfИсходный граф')
    Cycles = grCycleBasis(E); % все независимые циклы
%     for k1 = 1:size(Cycles,2)
%         grPlot(V,E(find(Cycles(:,k1)),:),'g','%d',''); % очередной цикл
%         set(get(gcf,'CurrentAxes'),...
%         'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
%         title(['\bfЦикл N' num2str(k1)]);
%     end
    end
    if isempty(Cycles)
    disp('В графе коллизий нет циклов.');
    add_delays(numRob(1,2),T2,S2,CollidedRobotsNum_S2);
    else
%         disp('NEXT');
        ActiveDist_2 = algorithm_2(Peak, Edge, numRob(1,2), S2, T2,...
                                 PathKor_S2,CollidedRobotsNum_S2);
    end

end

