function TargetKor = SettingTargetCube(Delta, TargetNum)
% ��������� ����������� ���� � ������ SKOR ���������� �������
% Delta - ��� �������
% M = fix(SKOR/Delta)+1 ����� ����� � ����
% TargetNum = 6*M^2-12*M+8 ����� ����� ��������� �����
% TargetKor - ���������� ��������� �����

global FIN_EDGE

TargetKor = zeros(TargetNum, 3); 
n = 1; % ������� ��������� �����
    % ���������� ������ �����
            for i = 1:Delta:(FIN_EDGE+1)
                for j = 1:Delta:(FIN_EDGE+1)
                        TargetKor(n,:) = [0, i-1, j-1];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'or','MarkerSize',MASHTAR);
                        n = n+1; 
                end
            end
    % ���������� ������ �����
            for i = (Delta+1):Delta:(FIN_EDGE+1)
                for j = 1:Delta:(FIN_EDGE+1)
                        TargetKor(n,:) = [i-1, 0, j-1];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'or','MarkerSize',MASHTAR);
                        n = n+1; 
                end
            end
    % ���������� ������� �����
            for i = (Delta+1):Delta:(FIN_EDGE+1)
                for j = (Delta+1):Delta:(FIN_EDGE+1)
                        TargetKor(n,:) = [i-1, j-1, 0];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'or','MarkerSize',MASHTAR);
                        n = n+1; 
                end
            end     
    % ���������� ��������� �����
            for i = (Delta+1):Delta:(FIN_EDGE+1)
                for j = (Delta+1):Delta:(FIN_EDGE+1)
                        TargetKor(n,:) = [FIN_EDGE, i-1, j-1];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'oc','MarkerSize',MASHTAR);
                        n = n+1; 
                end
            end 
    % ���������� ����� �����
            for i = (Delta+1):Delta:((FIN_EDGE+1)-Delta)
                for j = (Delta+1):Delta:(FIN_EDGE+1)
                        TargetKor(n,:) = [i-1, FIN_EDGE, j-1];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'oc','MarkerSize', MASHTAR);
                        n = n+1; 
                end
            end 
    % ���������� ������ �����
            for i = (Delta+1):Delta:((FIN_EDGE+1)-Delta)
                for j = (Delta+1):Delta:((FIN_EDGE+1)-Delta)
                        TargetKor(n,:) = [i-1, j-1, FIN_EDGE];                       
%                         plot3(TargetKor(n,1), ...
%                               TargetKor(n,2), ...
%                               TargetKor(n,3), ...
%                                 'oc','MarkerSize',MASHTAR);
                        n = n+1; 
                end
            end