function PRCurve(pPR, pObject, pClassifier, pDescription, withTestset)

figure(2)
atBeginning = 1;
for i=1:size(pPR, 1)
    if (pPR(i,5)==-1 && atBeginning==1)
        pPR(i,5) = 0;
    elseif (pPR(i,5)==-1 && atBeginning==0)
        pPR(i,5) = 1;
    else
        atBeginning = 0;
    end
end
plot([(pPR(:,5))], [pPR(:,3)], 'b', 'LineWidth', 1.25)
%hold on;
if (withTestset == 1)
    plot([(pPR(:,9))], [pPR(:,7)], 'r', 'LineWidth', 1.25)
end
set(gca, 'FontSize', 12);
set(gca, 'XTick', 0:0.1:1);
xlabel('Precision')
ylabel('Recall')
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