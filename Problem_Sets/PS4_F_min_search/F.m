%% Code for Problem Set 4 Exercise 5 (F_min search algorithm)
function s_T_bar = F(p_0)
% [s_T_bar] = F(p_0)
%
% Simulates the blimp dynamics and adjoint equations over the time horizon
% T and returns the final blimp state s(T).
%
% Input:
%   p_0             Initial adjoint value p(0), [4x1] vector.
%
% Output:
%   s_T_bar         Final blimp state s(T), [4x1] vector.

%% Problem parameters
global T s_0

%% Simulate system
[t, y] = ode45(@f_tilde, [0 T], [s_0; p_0]);

%% Extract final blimp state
s_T_bar = y(end,1:4)';

end