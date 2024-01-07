%% Code for Problem Set 2 Exercise 2.3 (Value Iteration Algorithm)
% Book: Dynamic Programming andOptimal Control by Dimitri P. Bertsekas, Vol. I, 3rd edition, 2005, 558 pages, hardcover
% Problem (BERTSEKAS, p. 446, exercise 7.3, Lecture 5)

%% clear workspace and command window
clear all;
close all;
clc;

% Define total number of iterations of VI algorithm
NUM_ITER = 1000;
% Stage costs g[state, admissible control input]
g = [4 6; -5 -3];
% Transition probabilities P[origin state, destination state, applied control input]
P = zeros(2,2,2);
% Advertise/do research (u=1):
P(:,:,1) = [0.8 0.2; 0.7 0.3];
% Don't advertise/don't do research (u=2):
P(:,:,2) = [0.5 0.5; 0.4 0.6];
% Discount factor
alpha = 0.99;

%% Value Iteration
costJ = [0,0];
costJnew = [0,0];
policy = [0,0];    

% Start Value Iteration
for k=1:NUM_ITER
    for i=1:2 
        [costJnew(i),policy(i)] = max( g(i,:) + alpha*costJ*squeeze(P(i,:,:)) );
    end;
    % For plot
    res_J(k,:) = costJnew;
    current = ['k=',num2str(k,'%5d'), 'J(1)=',num2str(costJnew(1),'%6.4f'),'J(2)=',num2str(costJnew(2),'%6.4f'),'mu(1)=',num2str(policy(1),'%3d'),'mu(2)=',num2str(policy(2),'%3d')];
    
    % Update cost
    costJ = costJnew;
    
    disp(current);

end;

% Display computed costs
disp('Result:');
disp(['J*(1) = ',num2str(costJ(1),'%9.6f')]);
disp(['J*(2) = ',num2str(costJ(2),'%9.6f')]);
disp(['mu*(1) = ',num2str(policy(1))]);
disp(['mu*(2) = ',num2str(policy(2))]);

% Plot costs over iterations.
kk = 1:size(res_J,1);

figure;
subplot(2,1,1);
plot(kk,[res_J(:,1)],[kk(1),kk(end)],[costJ(1), costJ(1)],'k');
grid;
legend('J_k(1)','J*(1)');
xlabel('iteration');

subplot(2,1,2);
plot(kk,[res_J(:,2)],[kk(1),kk(end)],[costJ(2), costJ(2)],'k');
grid;
legend('J_k(2)','J*(2)');
xlabel('iteration');


