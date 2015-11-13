function ROCCurve(pROC, pObject, pClassifier, pDescription, withTestset)

figure(1)
plot([1; (1-pROC(:,4))], [1; pROC(:,3)], 'b', 'LineWidth', 1.25)
%hold on;
if (withTestset == 1)
    plot([1; (1-pROC(:,8))], [1; pROC(:,7)], 'r', 'LineWidth', 1.25)
end
set(gca, 'FontSize', 12);
set(gca, 'XTick', 0:0.1:1);
xlabel('False Positive Rate')
ylabel('True Positive Rate')
xlim([0,1]);
ylim([0,1]);
grid on
if (~strcmp(pDescription, ''))
    if (strcmp(pObject, 'All'))
        title(strvcat([pClassifier, ' performance for all classes'], ['with ', pDescription]))
    else
        title(strvcat([pClassifier, ' performance for class "', pObject, '"'], ['with ', pDescription]))
    end
end
if (withTestset==1)
    legend('cross-validation', 'test set', 'Location', 'SouthEast')
end
%legend('boxoff')
end