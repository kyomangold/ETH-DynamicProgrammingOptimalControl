%% Code for Problem Set 4 Exercise 5 (F_min search algorithm)

%% Clear workspace and command window
clear all;
close all;
clc;

%% Parameters from problem statement
global T alpha_x alpha_z beta s_0 s_T

T = 1;          % Time horizon [h]
alpha_x = 5;    % Drag coefficient in x-direction [1/h]
alpha_z = 10;   % Drag coefficient in z-direction [1/h]
beta = 8;       % Wind speed constant [km/h]

s_0 = [-40;...  % Initial x-position [km]
      20;...    % Initial x-velocity [km/h]
      2;...     % Initial z-position [km]
      0];       % Initial z-velocity [km/h]
  
s_T = [0;...    % Final x-position [km]
      0;...     % Final x-velocity [km/h]
      0;...     % Final z-position [km]
      0];       % Final z-velocity [km/h]
  
%% Solve two-point boundary value problem
% Define residual function
f_res = @(p_0) norm(F(p_0)-s_T);

% Use fminsearch to find p_0_star that yields s(T) = s_T
[p_0_star, residual, flag, output] = fminsearch(@(p_0) f_res(p_0),zeros(4,1),optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));

%% Recover optimal solution
[t, y_star] = ode45(@f_tilde, [0 T], [s_0; p_0_star]);

s_star = y_star(:,1:4)';                % Optimal blimp state s*(t)
p_star = y_star(:,5:8)';                % Optimal costate p*(t)
u_star = [-p_star(2,:); -p_star(4,:)];  % Optimal control u*(t)
  
%% Display results
disp(' ');
disp('------------------------------------------------------------------');
disp('Result:');
if (flag == 1)      % found a minimum
    disp(['fminsearch found a solution in ',num2str(output.iterations),' iterations.']);
    disp(['The residual at the optimal p_0 is: ',num2str(residual),'.']);
    disp('The optimal position trajectory and control inputs are shown in the plots.');
    % Plot optimal blimp position trajectory
    figure(1);
    plot(s_star(1,:),s_star(3,:));
    axis([min(s_star(1,:)) max(s_star(1,:)) 0 1.1*max(s_star(3,:))]);
    title('Optimal Blimp Position Trajectory');
    xlabel('Position x*(t) [km]');
    ylabel('Height z*(t) [km]');

    % Plot corresponding optimal control inputs
    figure(2);
    plot(t, u_star(1,:), t, u_star(2,:),'r:');
    axis([0 T 1.1*min([min(u_star(1,:)),min(u_star(2,:))]) max([max(u_star(1,:)),max(u_star(2,:))])]);
    title('Optimal Control Inputs');
    xlabel('Time t [h]');
    ylabel('Control input u*(t) [km/h^2]');
    legend('u_1','u_2');

    findfigs;
elseif (flag == 0)  % reached maximum number of function evaluations or iterations
    disp('fminsearch did not find a minimum in the maximum number of function evaluations or iterations.');
else                % terminated otherwise
    disp('fminsearch could not find a minimum due to an error in the evaluation function.');
end 
disp(' ');
disp('------------------------------------------------------------------');
