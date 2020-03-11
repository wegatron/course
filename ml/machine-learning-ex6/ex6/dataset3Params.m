function [C, sigma] = dataset3Params(X, y, Xval, yval)
%DATASET3PARAMS returns your choice of C and sigma for Part 3 of the exercise
%where you select the optimal (C, sigma) learning parameters to use for SVM
%with RBF kernel
%   [C, sigma] = DATASET3PARAMS(X, y, Xval, yval) returns your choice of C and 
%   sigma. You should complete this function to return the optimal C and 
%   sigma based on a cross-validation set.
%

% You need to return the following variables correctly.

candicate_val = [0.01; 0.03; 0.1; 0.3; 1; 3; 10; 30];
min_error = 0;
C = 0.01;
sigma = 0.01;
for i=1:8
    cur_C = candicate_val(i);
    for j=1:8
        cur_sigma = candicate_val(j);
        % train
        model= svmTrain(X, y, cur_C, @(x1, x2) gaussianKernel(x1, x2, cur_sigma));
        % validate error
        predictions = svmPredict(model, Xval);
        verror = mean(double(predictions ~= yval));
        
        if((i==1 && j == 1) || min_error > verror)
            min_error = verror;
            C = cur_C;
            sigma = cur_sigma;
        end
    end
end


% ====================== YOUR CODE HERE ======================
% Instructions: Fill in this function to return the optimal C and sigma
%               learning parameters found using the cross validation set.
%               You can use svmPredict to predict the labels on the cross
%               validation set. For example, 
%                   predictions = svmPredict(model, Xval);
%               will return the predictions on the cross validation set.
%
%  Note: You can compute the prediction error using 
%        mean(double(predictions ~= yval))
%







% =========================================================================

end
