function new_coordinates = check_distance(distance, coordinates, num)
global min_dist
% x_1 = coordinates(:,1);
% y_1 = coordinates(:,2);
answer = true;
while answer
            if (distance < min_dist) && (distance~=0)
                x_1 = randi([1 16],1,num);
                y_1 = randi([1 20],1,num);
                new_coordinates = [x_1; y_1]';
                new_distance = pdist2(coordinates, coordinates);
            else
                answer = false;
            break
            end

end
end

