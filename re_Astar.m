clear all; 
close all;
clc;

%% Astar
% Map
map_ = zeros(40,40);
obstacle = [];
map_(1:6, 3) = inf;
map_(6:10, 6) = inf;
map_(1:6, 9) = inf;
map_(11,14:20) = inf;
map_(25:35,30) = inf;
map_(30:35,15:20) =inf;
map_(15:17,1:8) = inf;

width_ = size(map_,1);
height_ = size(map_,2);

hold all;
MAP1 = figure(1);
set(MAP1, 'Color', 'White')
surf(map_')
colormap(gray)
view(2)
grid on
axis([1 40 1 40])

start_pos = [1,1];
end_pos = [37,35];

path = []
% Heuristic
for x = 1:width_
    for y= 1:height_
        if map_(x,y) ~=inf
            H_score(x,y) = func_distance(end_pos(1), x, end_pos(2),y);
            G_score(x,y) = inf;
        else
            H_score(x,y) = inf;
        end
    end
end

G_score(start_pos(1), start_pos(2)) = 0;
F_score(start_pos(1), start_pos(2)) = H_score(start_pos(1), start_pos(2));

plot(start_pos(1), start_pos(2),'s', 'MarkerFaceColor','b');
plot(end_pos(1), end_pos(2),'s', 'MarkerFaceColor','r');

open_Node = [start_pos G_score(start_pos(1), start_pos(2)), F_score(start_pos(1), start_pos(2)), 0]; % start_x start_y G_score F_score parent_x parent_y
close_Node = [];

prev_pos = open_Node;

finish = 0;
pause(5)
%% 
while finish == 0

    [A, I] = min(open_Node(:,4));
    current_pos = open_Node(I,:);
    close_Node = [close_Node;current_pos]
    
    if current_pos(1) == end_pos(1) && current_pos(2) == end_pos(2)
        finish = 1;
        close_Node = [close_Node;current_pos];
    end
    open_Node = [];
    for x = current_pos(1)-1: current_pos(1)+1
        for y = current_pos(2)-1:current_pos(2)+1
            if x < 1 || x > width_
                continue
            end
            if y < 1 || y > height_
                continue
            end
            
            G_update =func_distance(current_pos(1),x,current_pos(2),y);
            G_score(x,y) = G_update;
            F_update(x, y) = G_score(x,y) + H_score(x,y);
            new_Node = [x y G_score(x,y) F_update(x,y) size(close_Node,1)];

%             if (new_Node(1) == prev_x) && (new_Node(2) == prev_y)
%                 new_Node(4) = inf;
%             end

            open_Node = [open_Node;new_Node];
            plot(open_Node(:,1), open_Node(:,2),'s','MarkerFaceColor','g')  
            
        end
    end
    
    for i = 1:(length(open_Node(:,1))-1)
        if (open_Node(i,1) == current_pos(1)) && (open_Node(i,2) == current_pos(2))
            open_Node(i,:) = [];
        end
        if (open_Node(i,1) == prev_pos(1)) && (open_Node(i,2) == prev_pos(2))
            open_Node(i,4) = inf;
        end
    end
    plot(current_pos(1), current_pos(2),'s','MarkerFaceColor','black')
    F_update(current_pos(1), current_pos(2)) = inf;
    prev_pos(1) = current_pos(1);
    prev_pos(2) = current_pos(2);
    pause(0.1)
%     finish =1;
end

j = size(close_Node,1);
j_save = [];

while(j > 0)
    x = close_Node(j,1);
    y = close_Node(j,2);
    j = close_Node(j,5);
    j_save = [ j; j_save];
    path = [x y;path];
end

for i = 1:size(path,1)
    plot(path(i:end,1),path(i:end,2),'b--o','LineWidth',3)
    pause(0.1)
end














