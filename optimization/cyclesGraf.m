function cyclesGraf(A,V,E)

if max(A(:)) == 1
    grPlot(V,E,'g','%d',''); % ������ ����
    set(get(gcf,'CurrentAxes'),...
    'FontName','Times New Roman Cyr','FontSize',10) % ���������� �����
    title('\bf�������� ����')
    Cycles = grCycleBasis(E); % ��� ����������� �����
    for k1 = 1:size(Cycles,2)
        grPlot(V,E(find(Cycles(:,k1)),:),'g','%d',''); % ��������� ����
        set(get(gcf,'CurrentAxes'),...
        'FontName','Times New Roman Cyr','FontSize',10) % ���������� �����
        title(['\bf���� N' num2str(k1)]);
    end
    if isempty(Cycles)
    disp('� ����� �������� ��� ������.');
    % ���������� ��������
    % �������� �2
    add_delays(num_of_robots,TargetAll);
end
end

