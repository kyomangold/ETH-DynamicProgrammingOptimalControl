%% Code for Problem Set 4 Exercise 5 (F_min search algorithm)
function y_dot = f_tilde(t,y)
% y_dot = f_tilde(t,y)
%
% First-order differential equation describing the evolution of the
% combined blimp state s and costate p:
%   y = (s p),
%   y_dot = f_tilde(y).
%
% Input:
%   y               Combined blimp state s and costate p, [8x1] vector.
%
% Output:
%   y_dot           Derivative of y describing the evolution over time,
%                   [8x1] vector.


%% Problem parameters
global alpha_x alpha_z beta

%% Split y into blimp state s and adjoint p
s = y(1:4);
p = y(5:8);

%% Compute optimal input
u_star = zeros(2,1);

u_star(1) = -p(2);
u_star(2) = -p(4);

%% Blimp dynamics
s_dot = zeros(4,1);

s_dot(1) = s(2);
s_dot(2) = -alpha_x * (s(2) - beta * s(3)^2) + u_star(1);
s_dot(3) = s(4);
s_dot(4) = -alpha_z * s(4) + u_star(2);

%% Adjoint equations
p_dot = zeros(4,1);

p_dot(1) = 0;
p_dot(2) = -p(1) + p(2) * alpha_x;
p_dot(3) = -2 * p(2) * alpha_x * beta * s(3);
p_dot(4) = -p(3) + p(4) * alpha_z;

%% Return y_dot
y_dot = [s_dot; p_dot];

end