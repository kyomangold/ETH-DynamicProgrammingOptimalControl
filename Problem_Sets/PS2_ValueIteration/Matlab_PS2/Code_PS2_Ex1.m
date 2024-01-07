%% Code for Problem Set 2 Exercise 1.2 (Value Iteration Algorithm)
% Book: Dynamic Programming andOptimal Control by Dimitri P. Bertsekas, Vol. I, 3rd edition, 2005, 558 pages, hardcover
% Problem (BERTSEKAS, p. 445, exercise 7.1, Lecture 4)

%% clear workspace and command window
clear all;
close all;
clc;

%% Given Parameters from Problem Statement
% Landing probability
p(2) = 0.95; % Slow serve: p_S

% Winning probability
q(1) = 0.6; % Fast serve: q_F
q(2) = 0.4; % Slow serve: q_S

% Define value iteration error convergence criteria
err = 1e-100;

% Define vector of incremental values for p(1)
p_incr_vec = 0 : 0.05 : 1;
prob_win_vec = zeros(1, length(p_incr_vec)); % probability of winning

%% Start Value Iteration
for p_incr = p_incr_vec
    % Landing probability
	p(1) = p_incr;
    % Initialize costs to 1
    J = ones(4, 4, 2);
    % Initialize the optimal control policy: 1 is Fast serve, 2 is Slow serve
    FVal = ones(4, 4, 2);
    % Initialize cost-to-go
    costToGo = zeros(4, 4, 2);
    % Iterate until cost converged
    counter = 0;
    while (1)
        counter = counter + 1;
        % Update value
        for i = 1:3
            [costToGo(4,i,1),FVal(4,i,1)] = max(q.*p + (1-q).*p.*J(4,i+1,1)+(1-p).*J(4,i,2));
            [costToGo(4,i,2),FVal(4,i,2)] = max(q.*p + (1-q.*p).*J(4,i+1,1));
            [costToGo(i,4,1),FVal(i,4,1)] = max(q.*p.*J(i+1,4,1) + (1-p).*J(i,4,2));
            [costToGo(i,4,2),FVal(i,4,2)] = max(q.*p.*J(i+1,4,1));
            for j = 1:3
                [costToGo(i,j,1),FVal(i,j,1)] = max(q.*p.*J(i+1,j,1) + (1-q).*p.*J(i,j+1,1)+(1-p).*J(i,j,2));
                [costToGo(i,j,2),FVal(i,j,2)] = max(q.*p.*J(i+1,j,1) + (1-q.*p).*J(i,j+1,1));
            end
        end
        [costToGo(4,4,1),FVal(4,4,1)] = max(q.*p.*J(4,3,1) + (1-q).*p.*J(3,4,1)+(1-p).*J(4,4,2));
        [costToGo(4,4,2),FVal(4,4,2)] = max(q.*p.*J(4,3,1) + (1-q.*p).*J(3,4,1));
        % Check if converged
        if (max(max(max(abs(J-costToGo))))/max(max(max(abs(costToGo)))) < err)
            J = costToGo;
            break;
        else
            J = costToGo;
        end
    end
    
    % Probability of player 1 winning the game
    prob_win_vec(p_incr == p_incr_vec)=J(1, 1, 1);
    
    % Display
    disp(['Iterations needed:',num2str(counter,'%9.0f'),' For p_F = ',num2str(p_incr,'%9.2f'),', winning probability is ',num2str(J(1, 1, 1),'%9.2f')]);
end

%% Plot
figure(1);
plot(p_incr_vec, prob_win_vec,'*-');
title('Probability of the server winning a game');
xlabel('p_F'); ylabel('Probability of winning');

