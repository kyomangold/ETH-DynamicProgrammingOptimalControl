%% Code for Problem Set 1 Exercise 8 (Dynamic Programming Algorithm)
% Book: Dynamic Programming andOptimal Control by Dimitri P. Bertsekas, Vol. I, 3rd edition, 2005, 558 pages, hardcover
% Problem (Terminating Process, BERTSEKAS, p. 54, exercise 1.8, Lecture 3)

%% clear workspace and command window
clear all;
close all;
clc;

%% Initialize variables
p_w = 1/3; % probability that wk = 1
num_states = 11; % states: x = 0, ..., 10 
num_inputs = 11; % inputs: 0 <= xk + uk <= 10
num_timesteps = 10; 
x = 0 : num_states - 1; % state vector 
J_opt = zeros( num_states, num_timesteps + 1 ); % optimal cost-to-go for each state and time step
u_opt = zeros( num_states, num_timesteps ); % optimal input for each state and time step (except final one)

%% Initialize Dynamic Programming (DP) algorithm
J_opt( : , end ) = x.^2; % gN(xN) = xN^2

%% Perform DP algorithm
for k = ( num_timesteps - 1 ) : -1 : 0 % start at k = N-1 and decrease to k = 0
    % Iterate over all possible states
    for i = 1 : num_states
        % Read state
       	xk = x( i );
        % Determine allowed inputs
        set_Uk = -xk : 10 - xk; % 0 <= xk + uk <= 10
        % Compute expected cost-to-go for each allowed input
        cost_to_go = zeros( 1, num_inputs );
        for m = 1 : num_inputs;
            % Read input
            uk = set_Uk( m );
            % Compute cost-to-go for this input
            x_ref = ( k - 5 )^2;
            cost_to_go( m ) = ( xk - x_ref )^2 + uk^2 + p_w * J_opt( ( xk + uk ) == x, k + 2 ) + ( 1 - p_w ) * J_opt( xk == x, k + 2 );
        end
        % Find minimal cost-to-go and corresponding optimal input
        [ J_opt( i, k + 1 ), index_u_opt ] = min( cost_to_go );
      	u_opt( i, k + 1 ) = set_Uk( index_u_opt );
    end
end

%% Print result (req: control systems toolbox)
printmat( J_opt, 'optimal cost-to-go','x=0 x=1 x=2 x=3 x=4 x=5 x=6 x=7 x=8 x=9 x=10','k=0 k=1 k=2 k=3 k=4 k=5 k=6 k=7 k=8 k=9 k=10' ); 
printmat( u_opt, 'optimal input','x=0 x=1 x=2 x=3 x=4 x=5 x=6 x=7 x=8 x=9 x=10','k=0 k=1 k=2 k=3 k=4 k=5 k=6 k=7 k=8 k=9' ); 

%% Visualize result
figure( 1 );

% Plot for all states & timesteps
k = 0 : num_timesteps;
[ p, q ] = meshgrid( k, x );
plot( p(:), q(:), 'ko', 'LineWidth', 2, 'MarkerSize', 10 ); hold on;

% Plot state transitions
x_plus_u = x + u_opt( : , 1 )';
for i = 1 : num_timesteps
    kk = [ i - 1, i ];
    x_plus_u = x + u_opt( : , i )';
    state_trans_handle = plot( kk, [ x; x_plus_u ], '-b', 'LineWidth', 2 ); hold on;
end

% Plot reference state
x_ref_handle = plot( k, ( k - 5 ).^2, '-r', 'LineWidth', 2 ); hold off;

% Add labels
axis( [ -1, num_timesteps + 1, x( 1 ) - 1, x( end ) + 1 ] );
xlabel( 'k', 'Interpreter', 'tex' );
ylabel( 'x_k', 'Interpreter', 'tex' );
legend( [x_ref_handle, state_trans_handle( 1 ) ], 'x_{ref}(k)', 'state transitions for w_k = 1', 'Location', 'NorthOutside' );



