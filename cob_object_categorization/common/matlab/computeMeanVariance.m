function [meanValue, varianceValue] = computeMeanVariance(A, numberClasses)
    folds = size(A,1)/numberClasses;
    B = reshape(A, numberClasses, folds)'
    meanValues = mean(B)
    meanValue = mean(meanValues)
    varianceValues = var(B)
    varianceValue = mean(varianceValues)
    stdValues = std(B)
    stdValue = mean(stdValues)
end