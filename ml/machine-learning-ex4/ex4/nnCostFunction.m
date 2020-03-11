function [J grad] = nnCostFunction(nn_params, ...
                                   input_layer_size, ...
                                   hidden_layer_size, ...
                                   num_labels, ...
                                   X, y, lambda)
%NNCOSTFUNCTION Implements the neural network cost function for a two layer
%neural network which performs classification
%   [J grad] = NNCOSTFUNCTON(nn_params, hidden_layer_size, num_labels, ...
%   X, y, lambda) computes the cost and gradient of the neural network. The
%   parameters for the neural network are "unrolled" into the vector
%   nn_params and need to be converted back into the weight matrices. 
% 
%   The returned parameter grad should be a "unrolled" vector of the
%   partial derivatives of the neural network.
%

% Reshape nn_params back into the parameters Theta1 and Theta2, the weight matrices
% for our 2 layer neural network
Theta1 = reshape(nn_params(1:hidden_layer_size * (input_layer_size + 1)), ...
                 hidden_layer_size, (input_layer_size + 1));

Theta2 = reshape(nn_params((1 + (hidden_layer_size * (input_layer_size + 1))):end), ...
                 num_labels, (hidden_layer_size + 1));

% Setup some useful variables
m = size(X, 1);
         
% You need to return the following variables correctly 
% J = 0;
% Theta1_grad = zeros(size(Theta1));
% Theta2_grad = zeros(size(Theta2));

% ====================== YOUR CODE HERE ======================
% Instructions: You should complete the code by working through the
%               following parts.
%
% Part 1: Feedforward the neural network and return the cost in the
%         variable J. After implementing Part 1, you can verify that your
%         cost function computation is correct by verifying the cost
%         computed in ex4.m
%
% Part 2: Implement the backpropagation algorithm to compute the gradients
%         Theta1_grad and Theta2_grad. You should return the partial derivatives of
%         the cost function with respect to Theta1 and Theta2 in Theta1_grad and
%         Theta2_grad, respectively. After implementing Part 2, you can check
%         that your implementation is correct by running checkNNGradients
%
%         Note: The vector y passed into the function is a vector of labels
%               containing values from 1..K. You need to map this vector into a 
%               binary vector of 1's and 0's to be used with the neural network
%               cost function.
%
%         Hint: We recommend implementing backpropagation using a for-loop
%               over the training examples if you are implementing it for the 
%               first time.

z2 = [ones(m,1) X] * Theta1'; % m x (l1+1) * (l1+1) x l2
a2 = sigmoid(z2);
z3 = [ones(m, 1) a2] * Theta2'; % m x l3 
h = sigmoid(z3);

y2 = zeros(m, num_labels);
y2(sub2ind(size(y2), 1:length(y), y')) = 1; % m \times 10
tmp = -y2 .* log(h) - (1-y2) .* log(1-h);
reg_theta1 = Theta1(:, 2:end);
reg_theta2 = Theta2(:, 2:end);
reg_theta = [reg_theta1(:); reg_theta2(:)];
J = 1/m * (sum(tmp(:)) + 0.5*lambda *(reg_theta' * reg_theta));

% Part 3: Implement regularization with the cost function and gradients.
%
%         Hint: You can implement this around the code for
%               backpropagation. That is, you can compute the gradients for
%               the regularization separately and then add them to Theta1_grad
%               and Theta2_grad from Part 2.
%

% -------------------------------------------------------------

% =========================================================================

% Unroll gradients
delta3 = h - y2; % m x l3 = 10
gz2 = sigmoidGradient(z2); % m x l2
delta2 = delta3 * Theta2(:, 2:end) .* gz2; % m x l2

grad_theta2 = delta3' * [ones(m,1) a2]; % l3 x m * m x l2
grad_theta1 = delta2' * [ones(m,1) X]; % l1 x m * m x l1

grad_lambda_theta1 = lambda * Theta1; grad_lambda_theta1(:, 1) = 0;
grad_lambda_theta2 = lambda * Theta2; grad_lambda_theta2(:, 1) = 0;
grad = 1/m * ([grad_theta1(:); grad_theta2(:)] + [grad_lambda_theta1(:); grad_lambda_theta2(:)]);
end
