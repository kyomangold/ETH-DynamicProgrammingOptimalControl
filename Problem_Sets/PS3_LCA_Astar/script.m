%% Code for Problem Set 3 Exercise 4 (Label Correcting method and A* algorithm)

%% Clear workspace and command window
clear all;
close all;
clc;

%% Initialize variables
load A.mat;     % Load matrix A that contains all the transition costs A(i,j) = a_ij to get from i to j.
N = length(A);  % Dimension of the problem: N = total number of nodes

%% Define start and terminal node
% Default values:
%   start_node = 1;
%   terminal_node = 100;
%
% Minimum path length (minimum total cost): 100
% Path: 1 -> 3 -> 41 -> 51 -> 100

start_node = 1;      
terminal_node = 100;

%% Label Correcting Algorithm
% Solve shortest path problem using the Label Correcting Algorithm
tic;
[optimal_cost1, optimal_path1] = lca(A,start_node,terminal_node);
time1 = toc;

%% A* Algorithm
% Solve shortest path problem using the A* Algorithm
tic;
[optimal_cost2, optimal_path2] = astar(A,start_node,terminal_node);
time2 = toc;

%% Display results
disp(' ');
disp('------------------------------------------------------------------');
disp('Results');
disp(['Problem with ',num2str(N),' nodes.']);
disp(['Optimal path from node ',num2str(start_node),' to ',num2str(terminal_node),':']);
disp('------------------------------------------------------------------');
disp(' ');
disp('Label Correcting Algorithm');
disp(['  Execution time: ',num2str(time1),'s.']);
disp(['  Minimum path length (minimum total cost): ',num2str(optimal_cost1)]);
disp(['  Path: ',num2str(optimal_path1)]);
disp(' ');
disp('A* Algorithm');
disp(['  Execution time: ',num2str(time2),'s  (',num2str(time2/time1),' times the time for method 1).']);
disp(['  Minimum path length (minimum total cost): ',num2str(optimal_cost2)]);
disp(['  Path: ',num2str(optimal_path2)]);
disp(' ');
disp('------------------------------------------------------------------');
