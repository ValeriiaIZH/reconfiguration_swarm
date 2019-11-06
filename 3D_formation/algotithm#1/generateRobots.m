function [ robots ] = generateRobots( location_mode, ROBOTNUM, maxPeak )
% location_mode = 1 - ������ ������ ������
% location_mode ~= 1 - ������ ��� �����������

    global MinDist;    
    shiftIn = 3;
    shiftOut = 6;

    bool = true;

    if (location_mode == 1)
% %     ������ "������" 
        randmin = (-1)*shiftIn*maxPeak;
        randmax = shiftIn*maxPeak;
        rand = false(ROBOTNUM, 3);
    else 
% %     ������ "���"
        randmin = (-1)*shiftIn*maxPeak;
        randmax = shiftOut*maxPeak;
        rand = logical(randi([0, 1], ROBOTNUM, 3));
    end  
    i = 0; 
    while bool
            % ���������� �������
         robots = randi([randmin, randmax], ROBOTNUM, 3); 
         robots(rand) = robots(rand)*(-1);
         % ��� ���� � ������� �����������: 
         a = false(size(robots));
         a(robots == 0)= true;
         robots(a) = 0.0001;

         % �������� �� �������� ����������
        DefDist = pdist(robots); % �������� ���������� ����� ��������
        if  min(DefDist) < MinDist 
            disp('Wrong distribution. Change configuration...');            
        else 
            bool = false;
        end
        i = i + 1;
        if i == 25
            disp('�� ������� ���������� ���������� ��������� ������������!');
            break;
        end
    end
end