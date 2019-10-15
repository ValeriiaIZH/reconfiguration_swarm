function cyclesGraf(A,V,E)

if max(A(:)) == 1
    grPlot(V,E,'g','%d',''); % рисуем граф
    set(get(gcf,'CurrentAxes'),...
    'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
    title('\bfИсходный граф')
    Cycles = grCycleBasis(E); % все независимые циклы
    for k1 = 1:size(Cycles,2)
        grPlot(V,E(find(Cycles(:,k1)),:),'g','%d',''); % очередной цикл
        set(get(gcf,'CurrentAxes'),...
        'FontName','Times New Roman Cyr','FontSize',10) % установили шрифт
        title(['\bfЦикл N' num2str(k1)]);
    end
    if isempty(Cycles)
    disp('В графе коллизий нет циклов.');
    % назначение задержек
    % АЛГОРИТМ №2
    add_delays(num_of_robots,TargetAll);
end
end

