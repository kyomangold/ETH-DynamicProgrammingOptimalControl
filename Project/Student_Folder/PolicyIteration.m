function [ J_opt, u_opt_ind ] = PolicyIteration(P, G)
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
    global TERMINAL_STATE_INDEX
    P_non_terminal = P;
    P_non_terminal(TERMINAL_STATE_INDEX, :, :) = [];
    P_non_terminal(:, TERMINAL_STATE_INDEX, :) = [];
    G_non_terminal = G;
    G_non_terminal(TERMINAL_STATE_INDEX, :) = [];
    % IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
    % in the ComputeTerminalStateIndex.m file (see main.m)

    %% Policy Iteration
    NUM_ITER = 1000;

    % Initialize a proper policy
    policy = 5*ones(K-1,1); 

    iter = 0;
    J_current = zeros(K-1,1);

    while iter <= NUM_ITER
        iter = iter + 1;
        
        % Stage 1 Policy Evaluation
        J_current = zeros(K-1,1);
        P_current = zeros(K-1,K-1);
        G_current = zeros(K-1,1);
        
        for i = 1:K-1
            for j = 1:K-1
                P_current(i,j) = P_non_terminal(i,j,policy(i));
            end
            G_current(i) = G_non_terminal(i,policy(i));
        end
        
        J_cor = (eye(K-1)-P_current)\G_current; 
        
        
        % Check if converged
        if J_cor == J_current
            continue
        else
            J_current = J_cor;
        end
        
        
        % Stage 2 Policy Improvement
        for i = 1:K-1
            summation = zeros(1,5);
            for j = 1:K-1
                for u = 1:5
                    summation(u) = summation(u) + P_non_terminal(i,j,u) * J_cor(j); 
                end
            end
            
            [~, policy(i)] = min(G_non_terminal(i,:)+summation);
        end

    end

    J_opt = J_current;
    J_opt = [J_opt(1:TERMINAL_STATE_INDEX - 1); 0; J_opt(TERMINAL_STATE_INDEX: end)];
    u_opt_ind = policy;
    u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX - 1); STAY; u_opt_ind(TERMINAL_STATE_INDEX: end)];
        
    end

        