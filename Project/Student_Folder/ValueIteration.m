function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
    %SOLUTION 
    %   Solve a stochastic shortest path problem by either Value Iteration,
    %   Policy Iteration, or Linear Programming.
    %
    %   [J_opt, u_opt_ind] = Solution(P, G) computes the optimal cost
    %   and the optimal control input for each state of the state space.
    %
    %   Input arguments:
    %       P:
    %           A (K x K x L)-matrix containing the transition probabilities
    %           between all states in the state space for all control inputs.
    %           The entry P(i, j, l) represents the transition probability
    %           from state i to state j if control input l is applied.
    %
    %       G:
    %           A (K x L)-matrix containing the stage costs of all states in
    %           the state space for all control inputs. The entry G(i, l)
    %           represents the cost if we are in state i and apply control
    %           input l.
    %
    %   Output arguments:
    %       J_opt:
    %       	A (K x 1)-matrix containing the optimal cost-to-go for each
    %       	element of the state space.
    %
    %       u_opt_ind:
    %       	A (K x 1)-matrix containing the index of the optimal control
    %       	input for each element of the state space. Mapping of the
    %       	terminal state is arbitrary (for example: STAY).
    
    global K STAY
    
    %% Handle terminal state
    % Do yo need to do something with the teminal state before starting policy
    % iteration ?
    global TERMINAL_STATE_INDEX
    % IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
    % in the ComputeTerminalStateIndex.m file (see main.m)
    
    %% Value Iteration
    % Define value iteration error bound and maximum iteration number
    err = 1e-5;
    NUM_ITER = 1000;

    % Our state space is S= K x K x L,
    % i.e. x_k = [start_state, end_state, input]

    % Initialize costs to 1 
    J = zeros(K, 1);

    % Initialize the optimal control policy: NORTH, any will do?
    policy = ones(K, 1);

    % Initialize cost-to-go
    CostToGo = zeros(K, 1);

    % Iterate until cost has converged
    iter = 0;

    while iter <= NUM_ITER
        
        % Increase counter
        iter = iter + 1;
        
        % Update the value
        for i = 1:K
            
            summation = zeros(1,5);
            for j = 1:K
                for u = 1:5
                    summation(u) = summation(u) + P(i,j,u) * J(j); 
                end
            end
            
            [CostToGo(i), policy(i)] = min(G(i,:) + summation);
        end
        
        % Check if cost has converged
        if (max(abs(J-CostToGo)))/(max(abs(CostToGo))) < err
            break;
        else
            % update cost
            J = CostToGo;
        end
    end

    policy(TERMINAL_STATE_INDEX) = 5;

    % output 
    J_opt = CostToGo;
    u_opt_ind = policy;
    end
    
    