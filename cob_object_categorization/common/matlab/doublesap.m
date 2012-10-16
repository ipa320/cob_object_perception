polyOrder = 2;
polyOrder2 = 4;
minLinesX2 = 2;
maxLinesX2 = 7;
recall_sapxx2yy4 = [73.4	78.2    75.1    78.2	77.7	78.1;  % SAP-4-4-2
			 76.1	79.3	78.0	77.9	78.8	79.2;  % SAP-5-5-2
			 77.6	79.6	79.1	78.9	78.3	79.9;  % SAP-6-6-2
			 78.2	79.1	78.0	79.9	79.9	78.8;  % SAP-7-7-2
			 76.8	79.8	78.9	79.7	79.0	79.0]; % SAP-8-8-2
         

figure(1)
set(gca, 'FontSize', 12)
plot(minLinesX2:maxLinesX2, recall_sapxx2yy4, 'LineWidth', 1.25)
grid
xlabel('Number of cuts in x and y-Direction, n_x=n_y')
ylabel('Recall (%)')
l__ = legend(['SAP-4-4-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-5-5-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-6-6-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-7-7-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], ['SAP-8-8-', num2str(polyOrder), '+x-x-', num2str(polyOrder2)], 'Location', 'SouthEast');    % b
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)