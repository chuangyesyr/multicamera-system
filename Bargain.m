% A function about bargain to guarantee convergence of Nash Equilibrium

function [P_i, Camera] = Bargain(tau, Object_label, N, Cameras, Utility_O)

% Pi is a set of probabilities
% tau is a small number between 0 and 1
% Object_label is the object that we are bargaining on
% N is the times of iteration

% Cameras = find(Table(:, Object_label) == 1);    % The cameras that can track this object
nC = length(Cameras);                 % The number of cameras that can track this object
if (nC >= 2)
    p_i = zeros(N, nC);
    Utility_head = zeros(N, nC);

    Utility_head(1,:) = 1/nC + rand(1,nC)*0.1;
    Utility_head(1,:) = Utility_head(1,:)/max(Utility_head(1,:));              % Predicted the initial object utilities for different camera

    for k = 1:N
        p_i(k, :) = exp(Utility_head(k, :)/tau)/sum(exp(Utility_head(k, :)/tau));
        index = find(p_i(k, :) == max(p_i(k, :)));
        Utility_head(k+1, :) = Utility_head(k, :);
%         Utility = Utility_O(Object_label, Cameras(index));                     % Should be normalized
        Utility = Utility_O(index);
        Utility_head(k+1, index) = Utility_head(k, index) + (Utility - Utility_head(k, index))/p_i(k, index);
    end
           
    P_i = p_i(N, :);
    Index = P_i == max(P_i);
    Camera = Cameras(Index);
else
    P_i = 1;
    Camera = Cameras;
end

end