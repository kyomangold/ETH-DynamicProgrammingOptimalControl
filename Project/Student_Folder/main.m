% main.m
%
% Matlab script that calls all the functions for computing the optimal cost
% and policy of the given problem.
%
% Dynamic Programming and Optimal Control
% Fall 2021
% Programming Exercise
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% --

%% Clear workspace and command window
clear all;
close all;
clc;

%% Options
% [M, N]
mapSize = [15, 15];
% Set to true to generate a random map of size mapSize, else set to false 
% to load the pre-exsisting example map
generateRandomWorld = false;

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
% IMPORTANT: Do not add or remove any global parameter in main.m
global GAMMA R Nc P_DISTRACTED
GAMMA  = 0.4;       % Resident gamma factor
R = 5;              % Resident range
Nc = 10;            % Time steps required to bring courrier to hospital when injured
P_DISTRACTED = 0.1; % probability that the courrier is distracted

% IDs of elements in the map matrix
global FREE BUILDING RESIDENT PIZZERIA DROP_OFF HOSPITAL POLICEMAN
FREE = 0;
BUILDING = 1;
RESIDENT = 2;
PIZZERIA = 3;
DROP_OFF = 4;
HOSPITAL = 5;
POLICEMAN = 6;

% Index of each action in the P and G matrices. Use this ordering
global NORTH SOUTH EAST WEST STAY
NORTH  = 1;
SOUTH = 2;
EAST = 3;
WEST = 4;
STAY = 5;

%% Generate map
% map(m,n) represents the cell type at indices (m,n) according to the axes
% specified in the PDF.
disp('Generate map');
if generateRandomWorld
	[map] = GenerateWorld(mapSize(1), mapSize(2));
else
    % We can load a pre-generated map.
    load('1010.mat');
end
MakePlots(map);

%% Generate state space
disp('Generate state space');
% Generate a (K x 3)-matrix 'stateSpace', where each accessible cell is
% represented by two rows (with and without carrying a package).
stateSpace = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) ~= BUILDING
            stateSpace = [stateSpace;
                          m, n, 0;
                          m, n, 1];
        end
    end
end
% State space size
global K
K=size(stateSpace,1);

%% Set the following to true as you progress with the files
terminalStateIndexImplemented = false;
transitionProbabilitiesImplemented = false;
stageCostsImplemented = false;
SolutionImplemented = false; 

%% Compute the terminal state index
global TERMINAL_STATE_INDEX
if terminalStateIndexImplemented
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end                  
%% Compute transition probabilities
if transitionProbabilitiesImplemented
    disp('Compute transition probabilities');
    % Compute the transition probabilities between all states in the
    % state space for all control inputs.
    % The transition probability matrix has the dimension (K x K x L), i.e.
    % the entry P(i, j, l) representes the transition probability from state i
    % to state j if control input l is applied.

    P = ComputeTransitionProbabilities(stateSpace, map);
end

%% Compute stage costs
if stageCostsImplemented 
    disp('Compute stage costs');
    % Compute the stage costs for all states in the state space for all
    % control inputs.
    % The stage cost matrix has the dimension (K x L), i.e. the entry G(i, l)
    % represents the cost if we are in state i and apply control input l.
    
    G = ComputeStageCosts(stateSpace, map);
end

%% Solve stochastic shortest path problem
if SolutionImplemented
    disp('Solve stochastic shortest path problem');
    
    %% Choose between Policy Iteration, Value Iteration and Linear Programming by (un-)commenting the lines accordingly
    % Policy Iteration
    [ J_opt, u_opt_ind ] = PolicyIteration(P, G);
    % Value Iteration 
    % [ J_opt, u_opt_ind ] = ValueIteration(P, G);
    % Linear Programming
    % [ J_opt, u_opt_ind ] = LinearProgramming(P, G);
    
    if size(J_opt,1)~=K || size(u_opt_ind,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
end

%% Plot results
disp('Plot results');
if SolutionImplemented
    MakePlots(map, stateSpace, J_opt, u_opt_ind, 'Solution');
end

%% Terminated
disp('Terminated');
