%% Code for Problem Set 3 Exercise 4 (Label Correcting method and A* algorithm)

function [optimal_cost, optimal_path] = lca(A,start_node,terminal_node)
% [optimal_cost, optimal_path] = lca(A,start_node,terminal_node)
%
% Executes Label Correcting algorithm (Book Dynamic Programming and Optimal
% Control, Bertsekes, page 81) using the depth-first method.
%
% Input:
%   A               [NxN] matrix, where the element A(i,j) = a_ij is the cost
%                   to move from node i to j.
%   start_node       Start node of desired shortest path, scalar from 1 to N.
%   terminal_node    Terminal node of desired shortest path, scalar from 1
%                   to N.
%
% Output:
%   optimal_cost         Cost of the shortest path(s), scalar:
%   optimal_path         Row vector containing the shortest path, e.g. 
%                   optimal_path = [1 33 45 43 79 100].

%% Initialization
N = length(A);      % N = total number of nodes

d = ones(1,N)*inf;  % d_i: Vector containing shortest path found so far 
d(start_node) = 0;

parent = ones(1,N)*inf; % Vector containing the parent of the shortest path found so far for each node.
parent(start_node) = 0;

candidates = zeros(1,N);  % Candidates list
last_candidate = 1;   
candidates(last_candidate) = start_node;

shortest_path = inf; % d_t

%% Check start and terminal node
if start_node == terminal_node
    optimal_cost = 0;
    optimal_path = [start_node terminal_node];
    return;
end

if (start_node > N || terminal_node > N)
    optimal_cost = inf;
    optimal_path = [start_node terminal_node];
    return;   
end


%% Start label correcting algorithm
while 1
    % STEP 1: Remove a node i from candidates and for each child j of i, execute STEP 2.
    i = candidates(last_candidate);
    candidates(last_candidate) = 0;
    last_candidate = last_candidate - 1;
    
    children = find(~isinf(A(i,:)) == 1);
    children(children == i) = [];
    
    for j = children
        % STEP 2: If d_i + a_ij < min(d_j,shortest_path), set d_j = d_i + a_ij and set i to be the parent of j.
        if (d(i) + A(i,j) < min(d(j),shortest_path))
            d(j) = d(i) + A(i,j);

            parent(j) = i;
            
            % If j ~= t, place j in candidates if it is not already in candidates, while if j == t, set shortest_path to the new value d_i + a_it of d_t
            if (j ~= terminal_node)
                if ~(candidates == j)
                    last_candidate = last_candidate + 1;
                    candidates(last_candidate) = j;
                end
            else
                shortest_path = d(j);
            end
        end
    end
    
    % STEP 3: If candidates is empty, terminate; else go to STEP 1.
    if (~last_candidate)
        break;
    end
end

% shortest_path is equal to the cost of the shortest path.
optimal_cost = shortest_path;

%% Construct shortest path
% Start at terminal node and, for each node, take parent node until start node reached
optimal_path = terminal_node;
while optimal_path(end) ~= start_node
    optimal_path(end + 1) = parent(optimal_path(end));
end
optimal_path = fliplr(optimal_path);  % Reverse path: start_node -> terminal_node

end