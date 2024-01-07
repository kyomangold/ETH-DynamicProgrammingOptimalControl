function P = ComputeTransitionProbabilities(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, controlSpace,
%   map, gate, mansion, cameras) computes the transition probabilities
%   between all states in the state space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 2)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE BUILDING RESIDENT POLICEMAN PIZZERIA DROP_OFF 
%                   HOSPITAL
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

    global GAMMA R P_DISTRACTED
    global RESIDENT PIZZERIA DROP_OFF HOSPITAL POLICEMAN
    global K
    
    L = 5;

     % Initialize P, assuming L = 5 control inputs
    P = zeros(K, K, L);

    % Loop over each state in stateSpace
    for From_state = 1:K
        for Control_input = 1:L
            % Extract the current state's position
            From_m = stateSpace(From_state, 1);
            From_n = stateSpace(From_state, 2);
            % Adjusted to two dimensions
            To_m = From_m + Increment(Control_input, 1);
            To_n = From_n + Increment(Control_input, 2);

            % Check if the new state is valid
            if To_m >= 1 && To_m <= size(map, 1) && To_n >= 1 && To_n <= size(map, 2) && ...
               map(To_m, To_n) ~= BUILDING
                To_state_index = find(ismember(stateSpace(:,1:2), [To_m, To_n], 'rows'));
                % Update transition probability for valid state
                P(From_state, To_state_index, Control_input) = ... % Probability value
            else
                % If not valid, handle as needed, e.g., stay in the same state
                P(From_state, From_state, Control_input) = ... % Probability value
            end
        end
    end
end

function valid = isValidState(m, n, map)
    % Get the size of the map
    [M, N] = size(map);

    % Check if the state is within the map boundaries
    if m < 1 || m > M || n < 1 || n > N
        valid = false;
        return;
    end

    % Define constants for terrain types
    FREE = 1; % Adjust these values to match your map's encoding
    BUILDING = 2;
    RESIDENT = 3;
    POLICEMAN = 4;
    PIZZERIA = 5;
    DROP_OFF = 6;
    HOSPITAL = 7;

    % Check if the state is not on a forbidden or restricted terrain
    forbiddenTerrains = [BUILDING, RESIDENT, POLICEMAN];
    if any(map(m, n) == forbiddenTerrains)
        valid = false;
        return;
    end

    % If none of the above conditions are met, the state is valid
    valid = true;
end

