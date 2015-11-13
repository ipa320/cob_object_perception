single = 0;

if (single==1)
    %time = load('../files/IPA2Data/timing.txt');
    %time = load('../files/IPA2Data/exp 18 - PCA3/oldmask/IPA2_Surf64Dev2_PCA3_timing.txt');
    %time = load('../files/IPA2Data/exp 22 - grsd/IPA2_RSD_GRSD_timing.txt');
    %time = load('../files/IPA2Data/exp 17 - vfh/vfh - sloppy mask - voxel filter 5mm - normtransition/IPA2_Surf64Dev2_vfh_timing.txt');
    %time = load('../files/IPA2Data/exp 2 - PCA3CF no pose normalization/PCA3CF7-7-2/IPA2_Surf64Dev2_PCA3CF7-7-2_timing.txt');
    %time = load('../files/IPA2Data/exp 22 - grsd/grsd - rtc/IPA2_RSD_GRSD_timing_2.txt');
    %time = load('../files/IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_timing.txt');
    %time = load('../files/IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_timing_pcanorm.txt');
    %time = load('../files/IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_timing_nonorm.txt');
    %time = load('../files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF7-7-2/IPA2_Surf64Dev2_PCA3CF7-7-2_timing.txt');
    time = load('../files/IPA2Data/exp 11 - double sap/PCA3CF5-5-2+5-5-4/IPA2_Surf64Dev2_PCA3CF5-5-2+5-5-4_timing.txt');
    

    meanPoints = mean(time(:,1))
    meanTime = mean(time(:,2))
    meanTime1 = mean(time(:,3:end))
    meanTime1 = sum(mean(time(:,3:end)))

    sprintf('throughput: %f', 1000000*meanPoints/meanTime)
else
    % a = normal SAP descriptor
    % b = for use with double SAP
    
    minLinesX = 4;
    maxLinesX = 8;
    minLinesX2 = 2;
    maxLinesX2 = 7;
    meanTime = [];
    %for polyOrder = 2:2:6              % a
    polyOrder = 2;                      % b
    polyOrder2 = 4;                     % b
    for linesX=minLinesX:maxLinesX      % b
        meanTimeRow = [];
        %for linesX=minLinesX:maxLinesX         % a
        for linesX2=minLinesX2:maxLinesX2       % b
            linesY2=linesX2;                    % b
            linesY=linesX;
            % file = ['../files/IPA2Data/exp 1 - PCA3CF/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_timing.txt'];
            % file = ['../files/IPA2Data/exp 2 - PCA3CF no pose normalization/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_timing.txt'];
            % file = ['../files/IPAData/Exp7 - PCA3CF/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/IPA_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_timing.txt'];
            % file = ['../files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_timing.txt'];
            file = ['../files/IPA2Data/exp 11 - double sap/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '+', num2str(linesX2), '-', num2str(linesY2), '-', num2str(polyOrder2), '/IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '+', num2str(linesX2), '-', num2str(linesY2), '-', num2str(polyOrder2), '_timing.txt']
            polyOrder;
            linesX;

            time = load(file);

            meanPoints = mean(time(:,1));
            meanTimeRow = [meanTimeRow, mean(time(:,2))]; %(1.0-0.3*linesX/8)*
            
%             if (polyOrder==6 && linesX==6)
%                 meanTimeRow(end) = meanTimeRow(end) + 1000;
%             end
%             if (polyOrder==6 && linesX==9)
%                 meanTimeRow(end) = meanTimeRow(end) + 1000;
%             end
%             if (polyOrder==6 && linesX==10)
%                 meanTimeRow(end) = meanTimeRow(end) + 12000;
%             end
            
%             if (polyOrder==2 && linesX>=5)
%                 meanTimeRow(end) = meanTimeRow(end) - 3000;
%             end
        end
        meanTime = [meanTime; meanTimeRow];
    end
    
    figure(1)
    set(gca, 'FontSize', 12)
    %plot(minLinesX:maxLinesX, meanTime/1000, 'LineWidth', 1.25)    % a
    plot(minLinesX2:maxLinesX2, meanTime/1000, 'LineWidth', 1.25)     % b
    grid
    xlabel('Number of cuts in x and y-Direction, n_x''=n_y''')
    ylabel('Computation Time (ms)')
    ylim([20,120])
    %l__ = legend('Order 2 Polynomial', 'Order 4 Polynomial', 'Order 6 Polynomial', 'Location', 'SouthEast');   % a
    l__ = legend(['SAP-4-4-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-5-5-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-6-6-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-7-7-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-8-8-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], 'Location', 'SouthEast');    % b
    set(l__, 'Box', 'off')
    set(l__, 'FontSize', 12)

    
    figure(2)
    set(gca, 'FontSize', 12)
    %plot(minLinesX:maxLinesX, 1000000*meanPoints./meanTime, 'LineWidth', 1.25)      % a
    plot(minLinesX2:maxLinesX2, 1000000*meanPoints./meanTime, 'LineWidth', 1.25)      % b
    grid
    xlabel('Number of cuts in x and y-Direction, n_x''=n_y''')
    ylabel('Throughput (points/s)')
    %l__ = legend('Order 2 Polynomial', 'Order 4 Polynomial', 'Order 6 Polynomial', 'Location', 'SouthWest');    % a
    l__ = legend(['SAP-4-4-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-5-5-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-6-6-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-7-7-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], ['SAP-8-8-', num2str(polyOrder), '+n_x''-n_y''-', num2str(polyOrder2)], 'Location',  'NorthEast');    % b
    set(l__, 'Box', 'off')
    set(l__, 'FontSize', 12)
end