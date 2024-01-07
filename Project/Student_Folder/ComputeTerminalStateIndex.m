function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE BUILDING RESIDENT PIZZERIA DROP_OFF HOSPITAL
%           REDLIGHT
%
%   Output arguments:
%
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF

% Find the coordinates of the DROP_OFF in the map
[dropOffRow, dropOffCol] = find(map == DROP_OFF);

% Assuming there is only one DROP_OFF in the map
dropOffCoordinates = [dropOffRow, dropOffCol, 0]; % The third column is a placeholder (e.g., for a package state)

% Find the index in stateSpace that matches the DROP_OFF coordinates
stateIndex = find(ismember(stateSpace(:, 1:2), dropOffCoordinates(1:2), 'rows'));

return
                  
end
