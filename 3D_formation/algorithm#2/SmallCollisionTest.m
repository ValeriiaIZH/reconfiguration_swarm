function [IsCol varargout] = SmallCollisionTest(t0, t1, StartPoint1, FinishPoint1, ...
                                            StartPoint2, FinishPoint2)
% ������� SmallCollisionTest ����������, ���� �� �������� ����� �����
% �������� �� ��������� ������� t0 � t1. StartPoint1, FinishPoint1 -
% ��������� � ��������� ����� ������� ������ ��������������; 
% StartPoint2, FinishPoint2 - ��������� � ��������� ����� ������� ������ 
% ��������������;
% IsCol = true, ���� �������� ����, IsCol = false, ���� �������� ���.
% Min - ����������� ���������� ����� ������������
%--------------------------------------------------------------------------

global R MINR VEL

if isequal(StartPoint1, FinishPoint1) && isequal(StartPoint2, FinishPoint2)
% ���� ��� ������ ���������� �� ������ ���������
    IsCol = false;    
else
    if isequal(StartPoint1, FinishPoint1) == false
       % ���� ������ ����� �������� �� ������ ���������
       F1 = VEL/pdist2(StartPoint1, FinishPoint1);   

       u1 = (FinishPoint1(1) - StartPoint1(1))*F1;
       v1 = (FinishPoint1(2) - StartPoint1(2))*F1;
       w1 = (FinishPoint1(3) - StartPoint1(3))*F1;

       A1 = StartPoint1(1) - u1*t0;
       B1 = StartPoint1(2) - v1*t0;
       C1 = StartPoint1(3) - w1*t0;
    else 
       % ���� ������ ����� ����������    
       u1 = 0;
       v1 = 0;
       w1 = 0;

       A1 = StartPoint1(1);
       B1 = StartPoint1(2);
       C1 = StartPoint1(3); 
    end
    
    if isequal(StartPoint2, FinishPoint2) == false 
       % ���� ������ ����� �������� �� ������ ���������  
       F2 = VEL/pdist2(StartPoint2, FinishPoint2);
       
       u2 = (FinishPoint2(1) - StartPoint2(1))*F2;
       v2 = (FinishPoint2(2) - StartPoint2(2))*F2;
       w2 = (FinishPoint2(3) - StartPoint2(3))*F2;

       A2 = StartPoint2(1) - u2*t0;
       B2 = StartPoint2(2) - v2*t0;
       C2 = StartPoint2(3) - w2*t0;
    else 
       % ���� ������ ����� ����������    
       u2 = 0;
       v2 = 0;
       w2 = 0;

       A2 = StartPoint2(1);
       B2 = StartPoint2(2);
       C2 = StartPoint2(3); 
    end
    
    % ��������� ����������� ������� ���������� ����� ������������
   a = (u1 - u2)^2 + (v1 - v2)^2 + (w1 - w2)^2;
   b = 2*(u1 - u2)*(A1 - A2) + ...
       2*(v1 - v2)*(B1 - B2) + ...
       2*(w1 - w2)*(C1 - C2);
   c = (A1 - A2)^2 + (B1 - B2)^2 + (C1 - C2)^2;
   D = b^2 - 4*a*c;
   
   if (abs(a) <= 0.001) 
       Min = 10*R^2;
            
   elseif (t0 > (-b/(2*a)))
       Min = (StartPoint1(1) - StartPoint2(1))^2 + ...
             (StartPoint1(2) - StartPoint2(2))^2 + ...
             (StartPoint1(3) - StartPoint2(3))^2;
         
   elseif (t1 < (-b/(2*a))) 
       Min = (FinishPoint1(1) - FinishPoint2(1))^2 + ...
             (FinishPoint1(2) - FinishPoint2(2))^2 + ...
             (FinishPoint1(3) - FinishPoint2(3))^2;   
         
   else
       Min = -D/(4*a);
   end
   
   % �������� �� ��������
   if Min < MINR^2 
       IsCol = true;
   else 
       IsCol = false;
   end
   MinDist = sqrt(Min);
end

varargout{1} = MinDist;





       
       
       
      
       
       
         
   
       
   
   
   
   
