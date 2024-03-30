close all
clear alclose all
clear all
clc

% Dimension of the workspace = config. space
max_rows = 7;
max_cols = 10;

% Highlighted black cells
highlightedCells = [
    1, 1;        % (1, 1)
    1, 4;        % (1, 4)
    1, 5;        % (1, 5)
    1, 10;       % (1, 10)
    2, 5;        % (2, 5)
    2, 10;       % (2, 10)
    3, 2;        % (3, 2)
    3, 3;        % (3, 3)
    3, 10;       % (3, 10)
    4, 3;        % (4, 3)
    5, 3;        % (5, 3)
    5, 6;        % (5, 6)
    5, 7;        % (5, 7)
    5, 8;        % (5, 8)
    6, 4;        % (6, 4)
    6, 5;        % (6, 5)
    6, 6;  ];    % (6, 6)
 

% Environment Definition
figure;
for row = 1:max_rows
    for col = 1:max_cols
        x = col - 1;            % Rectangular x coordinate
        y = max_rows - row;     % Rectangular y coordinate
        
        
        if ismember([row, col], highlightedCells, 'rows')   %Check if the cell is empty or highlighted
            rectangle('Position', [x, y, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k');    %If highlighted -> black cell
        else
            rectangle('Position', [x, y, 1, 1], 'FaceColor', 'w', 'EdgeColor', 'k');    %If not highlighted -> white cell
        end
    end
end

% "s" inside the starting cell in (4,1)
x_s = 0; % rectangular x coordinate starting from the bottom
y_s = 3; % rectangular y coordinate starting from the bottom
text(x_s + 0.5, y_s + 0.6, 's', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

% "g" inside the goal cell in (4,10)
x_g = 9; % rectangular x coordinate starting from the bottom
y_g = 3; % rectangular y coordinate starting from the bottom
text(x_g + 0.5, y_g + 0.6, 'g', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

axis equal;
axis tight;
grid on;

goal = [4, 10]; 
start = [4, 1]; 

%% 4-CELLS Algorithm

potential = inf(max_rows, max_cols);    % Set all the cell to infinite potential
potential(goal(1), goal(2)) = 0;        % Set the goal cell to zero
queue = goal;   % create a queue for the potential propagation

% Potential propagation
while ~isempty(queue)       %While the queue is not empty
    current = queue(1,:);   
    queue(1,:) = [];
    
    % Define neighbors cells (to be studied for the potential computation)m
    % it's in a matrix form.
    neighbors = [current(1)-1, current(2); % nord
                 current(1)+1, current(2); % sud
                 current(1), current(2)-1; % ovest
                 current(1), current(2)+1]; % est
    
    for i = 1:size(neighbors, 1)
        studied_row = neighbors(i, 1); 
        studied_col = neighbors(i, 2);
        
        
        if studied_row >= 1 && studied_row <= max_rows && studied_col >= 1 && studied_col <= max_cols   %Check if it's in the workspace
           
            if ~ismember([studied_row, studied_col], highlightedCells, 'rows') && potential(studied_row, studied_col) == inf  % Check if adjacent cell is empty and not yet visited
                potential(studied_row, studied_col) = potential(current(1), current(2)) + 1;    %Compute the potential of the studied cell under examination
                queue = [queue; studied_row, studied_col];  %Add the adjecent cell to the queue in order to be studied as well.
            end
        end
    end
end

disp('4-Cell Potentials Matrix');
disp('');
disp(potential);

% Plot of the workspace with all the potentials of the Navigation Function
% In order to make different plots with all the results, some commands are
% repeated.
figure;
for row = 1:max_rows
    for col = 1:max_cols
        x = col - 0.5;              % x coordinate of the center of the cell
        y = max_rows - row + 0.5;   % y coordinate of the center of the cell
        %The text will be put in the center

        
        if ismember([row, col], highlightedCells, 'rows') % If the cell is an obstacle 
            rectangle('Position', [col-1, max_rows-row, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k'); % black cell
        else
            % Write the potential in the plot only in cells with a finite number inside 
            if isfinite(potential(row, col))
                text(x, y, num2str(potential(row, col)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
            end
        end
    end
end

%Start
text(0.5, max_rows - 4 + 0.5, 's', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
%Goal
text(10 - 0.5, max_rows - 4 + 0.5, 'g', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

axis equal;
axis tight;
grid on;
title('4-Cell Workspace with potentials of the Navigation Function');

%% 4-Cell PATH Planning
current = start;
path = [current]; % it's a matrix containing all the point coordinates where to move.

while ~isequal(current, goal)
    neighbors = [current(1)-1, current(2); % nord
                 current(1)+1, current(2); % sud
                 current(1), current(2)-1; % ovest
                 current(1), current(2)+1; % est
                 current(1)-1, current(2)-1; % nord-ovest
                 current(1)-1, current(2)+1; % nord-est
                 current(1)+1, current(2)-1; % sud-ovest
                 current(1)+1, current(2)+1]; % sud-est
    
    
    min_potential = inf;
    next_cell = [];
    for i = 1:size(neighbors, 1)            % Select the cell with the lowest potential
        studied_row = neighbors(i, 1);
        studied_col = neighbors(i, 2);
        if studied_row >= 1 && studied_row <= max_rows && studied_col >= 1 && studied_col <= max_cols
            if potential(studied_row, studied_col) < min_potential
                min_potential = potential(studied_row, studied_col);
                next_cell = [studied_row, studied_col];
            end
        end
    end
    
    
    path = [path; next_cell];   % Add the lowest potential cell to the path.
    current = next_cell;        % Move to the next cell
end

% Draw the workspace with the path line
figure;
for row = 1:max_rows
    for col = 1:max_cols
        x = col - 0.5; 
        y = max_rows - row + 0.5;
        
        
        if ismember([row, col], highlightedCells, 'rows')
            rectangle('Position', [col-1, max_rows-row, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k'); % se è un ostacolo, disegna un rettangolo nero
        else
            if isfinite(potential(row, col))
                text(x, y, num2str(potential(row, col)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
            end
        end
    end
end

hold on;
plot(path(:, 2) - 0.5, max_rows - path(:, 1) + 0.5, 'r', 'LineWidth', 2); % Draw the optimal path
hold off;

text(start(2) - 0.5, max_rows - start(1) + 0.5, 's', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
text(goal(2) - 0.5, max_rows - goal(1) + 0.5, 'g', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

axis equal;
axis tight;
grid on;
title('4-Cell Algorithm Workspace with the Path of the Navigation Function');


%% 8-CELLS Algorithm
% Same thing of the 4-Cells algorithm, but in the while loop there are more
% cells to check

goal = [4, 10];
potential = inf(max_rows, max_cols);
potential(goal(1), goal(2)) = 0;

queue = goal;


while ~isempty(queue)
    current = queue(1,:);
    queue(1,:) = [];
    
    % Determina le celle adiacenti
    neighbors = [current(1)-1, current(2); % nord
                 current(1)+1, current(2); % sud
                 current(1), current(2)-1; % ovest
                 current(1), current(2)+1; % est
                 current(1)-1, current(2)-1; % nord-ovest
                 current(1)-1, current(2)+1; % nord-est
                 current(1)+1, current(2)-1; % sud-ovest
                 current(1)+1, current(2)+1]; % sud-est
    
    for i = 1:size(neighbors, 1)
        studied_row = neighbors(i, 1);
        studied_col = neighbors(i, 2);
        

        if studied_row >= 1 && studied_row <= max_rows && studied_col >= 1 && studied_col <= max_cols
            if ~ismember([studied_row, studied_col], highlightedCells, 'rows') && potential(studied_row, studied_col) == inf % Check if adjacent cell is empty and not yet visited
                potential(studied_row, studied_col) = potential(current(1), current(2)) + 1;  %Compute the potential of the studied cell under examination
                queue = [queue; studied_row, studied_col];  % Add to queue
            end
        end
    end
end

disp('8-Cell Potentials Matrix');
disp('');
disp(potential);

figure;
for row = 1:max_rows
    for col = 1:max_cols
        x = col - 0.5; 
        y = max_rows - row + 0.5;
        

        if ismember([row, col], highlightedCells, 'rows')
            rectangle('Position', [col-1, max_rows-row, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k'); 
        else
            if isfinite(potential(row, col))
                text(x, y, num2str(potential(row, col)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
            end
        end
    end
end

text(0.5, max_rows - 4 + 0.5, 's', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

text(10 - 0.5, max_rows - 4 + 0.5, 'g', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

axis equal;
axis tight;
grid on;
title('8-Cell Workspace with potentials of the Navigation Function');

%% 8-CELL PATH Planning
%Same of 4-CELL PATH Planning
current = start;
path = [current];

while ~isequal(current, goal)
   neighbors = [current(1)-1, current(2);           % nord
             current(1)+1, current(2);           % sud
             current(1), current(2)-1;           % ovest
             current(1), current(2)+1;           % est
             current(1)-1, current(2)-1;         % nord-ovest
             current(1)-1, current(2)+1;         % nord-est
             current(1)+1, current(2)-1;         % sud-ovest
             current(1)+1, current(2)+1];        % sud-est
    
    min_potential = inf;
    next_cell = [];
    for i = 1:size(neighbors, 1)
        studied_row = neighbors(i, 1);
        studied_col = neighbors(i, 2);
        

        if studied_row >= 1 && studied_row <= max_rows && studied_col >= 1 && studied_col <= max_cols
            if potential(studied_row, studied_col) < min_potential
                min_potential = potential(studied_row, studied_col);
                next_cell = [studied_row, studied_col];
            end
        end
    end

    path = [path; next_cell];   % Add the lowest potential cell in the path
    current = next_cell;
end


figure;
for row = 1:max_rows
    for col = 1:max_cols
        x = col - 0.5;
        y = max_rows - row + 0.5;
        
        if ismember([row, col], highlightedCells, 'rows')
            rectangle('Position', [col-1, max_rows-row, 1, 1], 'FaceColor', 'k', 'EdgeColor', 'k');
        else
            if isfinite(potential(row, col))
                text(x, y, num2str(potential(row, col)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
            end
        end
    end
end

hold on;
plot(path(:, 2) - 0.5, max_rows - path(:, 1) + 0.5, 'r', 'LineWidth', 2); 
hold off;

text(start(2) - 0.5, max_rows - start(1) + 0.5, 's', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');
text(goal(2) - 0.5, max_rows - goal(1) + 0.5, 'g', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Color', 'b');

axis equal;
axis tight;
grid on;
title('8-Cell Algorithm Workspace with the Path of the Navigation Function');