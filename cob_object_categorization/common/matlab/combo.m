emSteps = [20, 40, 50, 60, 80, 100, 200];
recall_emxxsap772 = [80.4, 79.7, 80.0, 80.4, 80.4, 79.7, 78.2];

figure(1)
set(gca, 'FontSize', 12)
plot(emSteps, recall_emxxsap772, 'LineWidth', 1.25)
grid
xlabel('Number of Clusters')
ylabel('Recall (%)')
xlim([emSteps(1), emSteps(end)])
ylim([65,100])
% l__ = legend(['SAP-4-4-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-5-5-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-6-6-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-7-7-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-8-8-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], 'Location', 'SouthEast');    % b
% set(l__, 'Box', 'off')
% set(l__, 'FontSize', 12)