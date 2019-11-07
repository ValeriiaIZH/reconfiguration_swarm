% ТЕСТИРОВАНИЕ АЛГОРИТМА "РАЗДЕЛЯЙ И ВЛАСТВУЙ"
% РЕКУРСИВНАЯ ФУНКЦИЯ
% TODO : пофиксить цикл while;
%        привести в нормальный вид;
%        спрятать в функцию
clc;
clearvars;
close all;
% тест алгоритма "разделяй и властвуй"
global MinDist Density NumOfPoints_1 Vel TIMESTEP MASHROB MINR R;
Density = 10;       % плотность размещения
MinDist = 1;        % минимальное допустимое расстояние
Vel = 1;            % скорость робота
TIMESTEP = 1;       % шаг дискретизации по времени 
MASHROB = 5;        % размер робота на графике
R = 1;              % радиус робота
MINR = R;           % минимальное расстояние между роботами
% назначение координат роботам(точкам)
NumOfPoints_1 = 16; % количество роботов
x_1 = randi([15 35],1,NumOfPoints_1);
y_1 = randi([15 35],1,NumOfPoints_1);
robotCoordinates = [x_1; y_1]';
bool1 = true;
dist1=pdist(robotCoordinates);
m = length(dist1);
while bool1
    if min(dist1)<MinDist
        x_1 = randi([15 35],1,NumOfPoints_1);
        y_1 = randi([15 35],1,NumOfPoints_1);
        robotCoordinates = [x_1; y_1]';
        dist1=pdist(robotCoordinates);
    else
        bool1 = false;
    break
    end
end

if min(dist1)<MinDist
    disp('error configuration');
end
% таргетные точки (фиксированы)- поверхность плоский квадрат
x_2 = [20 20 30 30];
y_2 = [20 30 30 20];

Peak = [x_2; y_2]';
      
Edge = [1 2;
        1 3;
        1 4;
        2 3;
        2 4;
        3 4];

NumOfPeak = length(Peak);
NumOfEdge = length(Edge);

AdjacentPeaks = zeros(NumOfPeak, 2);
for i = 1 : NumOfPeak
    [row, ~] = find(Edge == i, 2);
    AdjacentPeaks(i, :) = row';
end
clear row;

[ TargetAll, TargetPeak , TargetEdge ] = ...
                                         generateTargets_flat(Peak, Edge);
                                     
% уравнение прямой
[x_c, y_c, sol, F, M] = test_divideAndRule(robotCoordinates, TargetAll);
figure  
plot(M(:,1),M(:,2),'ko')
hold on
XL = get(gca,'XLim');
plot(x_1, y_1,'md', x_2, y_2,'b.');
grid on;
plot(x_c, y_c, 'r*');
plot([XL(1),XL(2)],[F(XL(1),sol(1),sol(2)),F(XL(2),sol(1),sol(2))],'LineWidth',1)
xLine1 = XL(1);
xLine2 =  XL(2);
yLine1 = F(XL(1),sol(1),sol(2));
yLine2 = F(XL(2),sol(1),sol(2));
coordLine = [xLine1 xLine2; yLine1 yLine2];
% расчёт коэффициентов функции (прямая f(x)=k*x+b)
A = [coordLine(1,1) 1; coordLine(1,2) 1];
B = [coordLine(2,1); coordLine(2,2)];
K = linsolve(A, B);
clear A B;
count1 = 0; % точки выше прямой
count2 = 0; % точки ниже прямой
count3 = 0; % точки на прямой
count4 = 0; 
count5 = 0; 
count6 = 0; 
for i = 1:NumOfPoints_1
if K(1)*x_1(i) + K(2) < y_1(i)
    count1 = count1 + 1;
    x_s1_test_r(i) = x_1(i);
    y_s1_test_r(i) = y_1(i);
end
if K(1)*x_1(i) + K(2) > y_1(i)
    count2 = count2 + 1;
    x_s2_test_r(i) = x_1(i);
    y_s2_test_t(i) = y_1(i);
end
if K(1)*x_1(i) + K(2) == y_1(i)
    count3 = count3 + 1;
end
if K(1)*TargetAll(i,1) + K(2) < TargetAll(i,2)
    count4 = count4 + 1;
    x_s1_test_T(i) = TargetAll(i,1);
    y_s1_test_T(i) = TargetAll(i,2);
end
if K(1)*TargetAll(i,1) + K(2) > TargetAll(i,2)
    count5 = count5 + 1;
    x_s2_test_T(i) = TargetAll(i,1);
    y_s2_test_T(i) = TargetAll(i,2);
end
if K(1)*TargetAll(i,1) + K(2) == TargetAll(i,2)
    count6 = count6 + 1;
end 
end
if (count1~=count4 && count2~=count5)
    bool2 = true;
    iteration = 1;
    while bool2
        [x_c, y_c, sol, F, M] = test_divideAndRule(robotCoordinates, TargetAll);
        clear A B count1 count2 count3 count4 count5 count6;
        plot(M(:,1),M(:,2),'ko')
        hold on
        XL = get(gca,'XLim');
        plot(x_1, y_1,'md', x_2, y_2,'b.');
        grid on;
        plot(x_c, y_c, 'r*');
        plot([XL(1),XL(2)],[F(XL(1),sol(1),sol(2)),F(XL(2),sol(1),sol(2))],'LineWidth',1)
        xLine1 = XL(1);
        xLine2 =  XL(2);
        yLine1 = F(XL(1),sol(1),sol(2));
        yLine2 = F(XL(2),sol(1),sol(2));
        coordLine = [xLine1 xLine2; yLine1 yLine2];
        % расчёт коэффициентов функции (прямая f(x)=k*x+b)
    A = [coordLine(1,1) 1; coordLine(1,2) 1];
    B = [coordLine(2,1); coordLine(2,2)];
    K = linsolve(A, B);
    
    count1x = 0; % точки выше прямой
    count2x = 0; % точки ниже прямой
    count3x = 0; % точки на прямой
    count4x = 0; 
    count5x = 0; 
    count6x = 0; 
    for i = 1:NumOfPoints_1
    if K(1)*x_1(i) + K(2) < y_1(i)
        count1x = count1x + 1;
    end
    if K(1)*x_1(i) + K(2) > y_1(i)
        count2x = count2x + 1;
    end
    if K(1)*x_1(i) + K(2) == y_1(i)
        count3x = count3x + 1;
    end
    if K(1)*TargetAll(i,1) + K(2) < TargetAll(i,2)
        count4x = count4x + 1;
    end
    if K(1)*TargetAll(i,1) + K(2) > TargetAll(i,2)
        count5x = count5x + 1;
    end
    if K(1)*TargetAll(i,1) + K(2) == TargetAll(i,2)
        count6x = count6x + 1;
    end 
    end
        iteration = iteration + 1;
        if iteration == 51
            msg = sprintf('too many iterations! %d',iteration);
            disp(msg);
            disp('Невозможно применить алгоритм "Разделяй и всластвуй для этой конфигурации"');
            break;
        end
        if count1x==count4x && count2x==count5x
            bool2 = false;
        end
    end 
%     if count3~=0 && count6~=0 && count3x~=0 && count6x~=0
%         disp('Точки на прямой!');
%     end
else
    disp('Деление множества на 2 подмножества');
end

% множество 1
S1_test = [x_s1_test; y_s1_test]';
S1_rob = [x_s1_test_r;y_s1_test_r]';
S1_rob( ~any(S1_rob,2), : ) = [];
S1_tar = [x_s1_test_T;y_s1_test_T]';
S1_tar( ~any(S1_tar,2), : ) = [];
S1 = [S1_rob; S1_tar];
% множество 2
S2_rob = [x_s1_test_r;y_s1_test_r]';
S2_rob( ~any(S2_rob,2), : ) = [];
S2_tar = [x_s1_test_T;y_s1_test_T]';
S2_tar( ~any(S2_tar,2), : ) = [];
S2 = [S2_rob; S2_tar];
%% вывод полученных значений
% msg = sprintf('Количество роботов выше прямой = %6.4g', count1); 
% disp(msg);
% msg = sprintf('Количество роботов ниже прямой = %6.4g', count2); 
% disp(msg);
% msg = sprintf('Количество роботов на прямой = %6.4g', count3); 
% disp(msg);
% msg = sprintf('Количество целей выше прямой = %6.4g', count4); 
% disp(msg);
% msg = sprintf('Количество целей ниже прямой = %6.4g', count5); 
% disp(msg);
% msg = sprintf('Количество целей на прямой = %6.4g', count6); 
% disp(msg);

