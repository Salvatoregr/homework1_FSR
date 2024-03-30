close all
clear all
clc

% Load map
load('image_map.mat');

% Define start and goal points
q_s = [30 125];
q_g = [135 400];

figure
imshow(image_map,'InitialMagnification','fit');     % To display a bigger plot of the environment
hold on
plot(q_s(2), q_s(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);   % Represented as a green circle (go)
plot(q_g(2), q_g(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % Represented as a red circle (ro)
title('RRT Path Planning');
hold off


% Parameters
max_iterations = 200;       % Max. iterations number
goal_radius = 15;           % Radius within which we consider the goal reached (like an error epsilon we allow to be)
delta = 20;                 % Step size for each iteration (distance from the q_near)
plot_delay = 0.1;           % Delay between iterations (in seconds) to make the graph animated

% Tree definition
tree = zeros(max_iterations, 2);    % Each row represents a node: (x,y)
tree(1, 1:2) = q_s;                 % The tree starts in q_s
node_number = 1;                    % Number of nodes added to the tree

% Plot the map
figure;
imshow(image_map,'InitialMagnification','fit');     % To display a bigger plot of the environment
hold on;

% Plot the Start and Goal points inside the map
plot(q_s(2), q_s(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); 
plot(q_g(2), q_g(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);  
title('RRT Path Planning');

actual_iterations = 0;  % Initialize actual iterations count

% RRT Algorithm loop
goal_reached = false; % Flag to indicate if the goal is reached

while node_number < max_iterations && ~goal_reached
    % Random point
    q_rand = [randi(size(image_map, 1)), randi(size(image_map, 2))];
    
    % Nearest point in the tree
    distances = sqrt(sum((tree(1:node_number, 1:2) - q_rand).^2, 2));
    [~, index] = min(distances);      % The first element is not necessary so we can avoid to store it
    q_near = tree(index, 1:2);        % Add a q_near inside the tree
    
    direction = (q_rand - q_near) / norm(q_rand - q_near);     % Find a direction of movement and move towards it with a delta step
    q_new = q_near + delta * direction;
    
    % Check if q_new is within map bounds and not in an obstacle
    if q_new(1) < 1 || q_new(1) > size(image_map, 1) || q_new(2) < 1 || q_new(2) > size(image_map, 2) || image_map(round(q_new(1)), round(q_new(2))) == 0
        continue;       % Restart the while loop -> q_new is invalid
    end
    
    
    line_points = [linspace(q_near(1), q_new(1), 100); linspace(q_near(2), q_new(2), 100)]';
    if any(image_map(sub2ind(size(image_map), round(line_points(:,1)), round(line_points(:,2)))) == 0) % Check if, after the path is generated, an obstacle is hit
        continue;       % Restart the while loop -> the path hits one or more obstacles
    end
    
    
    node_number = node_number + 1;  % Increment node count
    tree(node_number, 1:2) = q_new; % If everything is ok, add q_new to the tree

    % Plot the new point 
    plot(q_new(2), q_new(1), 'bo');
    plot([q_near(2), q_new(2)], [q_near(1), q_new(1)], 'b'); %connect it to the current point (q_near)
    drawnow; % Force the update of the plot
    
    % Add delay
    pause(plot_delay);
    
    % Check if goal is reached
    if norm(q_new - q_g) < goal_radius
        disp('Goal reached!');
        goal_reached = true;
    end
end

if ~goal_reached
    disp('No solution found, try adding more iterations');
end

