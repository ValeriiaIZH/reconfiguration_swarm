function [  IntersectedSeg ] = intersectingPaths( TimeSegments, RobotIndx )
% все работает ! TimeSegments - массив временных отрезков; 
% IntersectedSeg - номера пересекающихся временных отрезков; 
% RobotIndx - номера роботов,которым соответствуют временные отрезки TimeSegments.

% TimeSegments = cat(1, TimePoints(:, 1:2), TimePoints(:, 2:3), TimePoints(:, 3:4), TimePoints(:, 4:5));
% RobotIndx    = repmat((1 : NumOfRobots)', size(TimePoints, 2), 1);

ovlp         = @(x, y)bsxfun(@ge, x(:, 1), y(:, 1)') & bsxfun(@le, x(:, 1), y(:, 2)');
idx          = ovlp(TimeSegments, TimeSegments);
[row, col]   = ind2sub(size(idx), find(idx));
IntersectedSeg = [row, col];

% % 1 
% idx = bsxfun(@le, row, col);  
% IntersectedSeg(idx, :) = [];
% row(idx, :) = [];
% col(idx, :) = [];
% 
% idx = RobotIndx(row) == RobotIndx(col);
% IntersectedSeg(idx, :) = [];

% % 2 
% idx1 = bsxfun(@le, row, col);  
% idx2 = RobotIndx(row) == RobotIndx(col);
% idx = idx1 | idx2; 
% IntersectedSeg(idx, :) = [];

% % 3 
idx = bsxfun(@le, row, col) | (RobotIndx(row) == RobotIndx(col));
IntersectedSeg(idx, :) = [];

if max(max(IntersectedSeg)) <= intmax('uint16')
    IntersectedSeg = uint16(IntersectedSeg);
else 
    IntersectedSeg = uint32(IntersectedSeg);
end

