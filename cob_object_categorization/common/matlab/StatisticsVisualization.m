clc
close all
clear all

SingleClassVisualization = 0;
Object = 'ball';

%%%%% ---------- ALOI Database
%Objects = {'ball', 'bottle', 'box', 'cup', 'figure', 'jar', 'na', 'soft_toy'}

%%%%% ----- 8 bit SIFT descriptor, 256 bins histogram
% Folder = '../files/ALOI4Data/Classifier/Statistics/ROCCurve_256bins_RTC_';
% Description = 'ALOI4, FP2x2, 8bit SIFT descriptor -> 256bins histogram';
% Classifier = 'Random Trees Classifier';

%%%%% ----- 128 dimensional SIFT descriptor, 50 cluster histogram
% Folder = '../files/ALOI4Data/Classifier/Statistics/ROCCurve_128BitEM50_RTC_';
% Description = 'ALOI4, 128dim SIFT descriptor, 50 cluster histogram';
% Classifier = 'Random Trees Classifier';

%%%%% --------- IPA Database
Objects = {'ball', 'book', 'bottle', 'coffeepot', 'cuddly', 'cup', 'flowerpot', 'tetrapack', 'vehicle'}

performanceMatrix = [];

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_SurfEM50_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 50 cluster histogram';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_SurfEM50_HP_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 50 cluster histogram + PCA';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM300PCA3_DRelease_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 300 cluster histogram + PCA';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM300PCA3_SD_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 300 cluster histogram + PCA';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM184PCA3CF12FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 184 cluster histogram + 3 PCA + Curve fitting 3x4 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM046PCA3CF12FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 46 cluster histogram + 3 PCA + Curve fitting 3x4 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM184_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 184 cluster histogram';
% Classifier = 'Random Trees Classifier';


% ----------------------- new -----------------------------------

% Folder = '../files/IPAData/Exp1 - RGBI+ChannelLabel Descriptor/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM046PCA3CF30FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 46 cluster histogram + 3 PCA + Curve fitting 6x5 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp2 - Grayscale Descriptor/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM046PCA3CF30FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 46 cluster histogram + 3 PCA + Curve fitting 6x5 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp3 - RGBI Descriptor/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM046PCA3CF30FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 46 cluster histogram + 3 PCA + Curve fitting 6x5 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp4 - HSVI+ChannelLabel Descriptor/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM046PCA3CF30FS6_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 46 cluster histogram + 3 PCA + Curve fitting 6x5 + FP frame direction statistics 6';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp10 - EMPCA3CF on Surf64/Classifier/Statistics/ROCCurve_IPA_Surf64_EM200PCA3CF6-6-2_RTC_';
% Description = 'IPA, Surf descriptor, 200 cluster histogram + 3 PCA + Curve fitting 6-6-2';
% Classifier = 'Random Trees Classifier';

% % for em comparison
% for em=[46, 75, 100, 150, 200, 250]
% Folder = '';
% if (em < 100)
%     Folder = ['../files/IPAData/Exp6 - EM/EM0', num2str(em), '/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM0', num2str(em), '_RTC_'];
% else
%     Folder = ['../files/IPAData/Exp6 - EM/EM', num2str(em), '/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM', num2str(em), '_RTC_'];
% end
% Description = 'IPA, Surf + 3DEnv histogram descriptor';
% Classifier = 'Random Trees Classifier';

% for curve fitting comparison
minLinesX = 2;
maxLinesX = 10;
for polyOrder=2:2:6
for linesX=minLinesX:maxLinesX
linesY=linesX;
% with PCA pose normalization:
% Folder = ['../files/IPAData/Exp7 - PCA3CF/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
% without pose normalization: 
% Folder = ['../files/IPAData/Exp13 - PCA3CF-nonorm/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
% with roll pose normalization:
Folder = ['../files/IPAData/Exp15 - PCA3CF - roll pose normalization/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
Description = ['IPA, 3 PCA + Curve fitting ', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder)];
Classifier = 'Random Trees Classifier';
polyOrder
linesX


% Folder = '../files/IPAData/Exp6 - EM/EM250/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM250_RTC_';
% Description = ''; %Description = 'IPA, bag of local features descriptor, 250 cluster histogram';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp7 - PCA3CF/PCA3CF6-6-2/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_PCA3CF6-6-2_RTC_';
% Description = ['IPA, 3 PCA + Curve fitting 6-6-2'];
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp8 - Comparison local features/RGBI/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM200_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 200 cluster histogram';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Exp9 - EMPCA3CF on Surf64Dev2/EM200PCA3CF6-6-2/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_EM200PCA3CF6-6-2_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 200 cluster histogram + 3 PCA + Curve fitting 6-6-2';
% Classifier = 'Random Trees Classifier';

% Folder = '../files/IPAData/Classifier/Statistics/ROCCurve_IPA_Surf64Dev2_PCA3CF6-6-2_nonorm_RTC_';
% Description = 'IPA, Surf + 3DEnv descriptor, 200 cluster histogram + 3 PCA + Curve fitting 6-6-2';
% Classifier = 'Random Trees Classifier';

%%%%% assembled curve from several files
% ROC_RGBI = load('roc_rgbi.mat');
% ROC_CF = load('roc_cf.mat');
% ROC_COMBO = load('roc_combo.mat');
% 
% figure
% set(gca, 'FontSize', 16)
% plot([1; (1-ROC_RGBI.ROCAll(:,4))], [1; ROC_RGBI.ROCAll(:,3)], 'r', 'LineWidth', 1.5)
% hold on
% plot([1; (1-ROC_CF.ROCAll(:,4))], [1; ROC_CF.ROCAll(:,3)], 'g', 'LineWidth', 1.5)
% plot([1; (1-ROC_COMBO.ROCAll(:,4))], [1; ROC_COMBO.ROCAll(:,3)], 'b', 'LineWidth', 1.5)
% xlabel('False Positive Rate')
% ylabel('True Positive Rate (Recall)')
% xlim([0,1]);
% ylim([0,1]);
% grid on
% l__ = legend('local descriptors only', 'SAP descriptors only', 'both descriptor types', 'Location', 'SouthEast')
% set(l__, 'Box', 'off')
% 
% figure
% set(gca, 'FontSize', 16)
% atBeginning = 1;
% for i=1:size(ROC_RGBI.ROCAll, 1)
%     if (ROC_RGBI.ROCAll(i,5)==-1 && atBeginning==1)
%         ROC_RGBI.ROCAll(i,5) = 0;
%     elseif (ROC_RGBI.ROCAll(i,5)==-1 && atBeginning==0)
%         ROC_RGBI.ROCAll(i,5) = 1;
%     else
%         atBeginning = 0;
%     end
% end
% plot([(ROC_RGBI.ROCAll(:,5))], [ROC_RGBI.ROCAll(:,3)], 'r', 'LineWidth', 1.5)
% hold on
% for i=1:size(ROC_CF.ROCAll, 1)
%     if (ROC_CF.ROCAll(i,5)==-1 && atBeginning==1)
%         ROC_CF.ROCAll(i,5) = 0;
%     elseif (ROC_CF.ROCAll(i,5)==-1 && atBeginning==0)
%         ROC_CF.ROCAll(i,5) = 1;
%     else
%         atBeginning = 0;
%     end
% end
% plot([(ROC_CF.ROCAll(:,5))], [ROC_CF.ROCAll(:,3)], 'g', 'LineWidth', 1.5)
% for i=1:size(ROC_COMBO.ROCAll, 1)
%     if (ROC_COMBO.ROCAll(i,5)==-1 && atBeginning==1)
%         ROC_COMBO.ROCAll(i,5) = 0;
%     elseif (ROC_COMBO.ROCAll(i,5)==-1 && atBeginning==0)
%         ROC_COMBO.ROCAll(i,5) = 1;
%     else
%         atBeginning = 0;
%     end
% end
% plot([(ROC_COMBO.ROCAll(:,5))], [ROC_COMBO.ROCAll(:,3)], 'b', 'LineWidth', 1.5)
% xlabel('Precision')
% ylabel('Recall')
% xlim([0,1]);
% ylim([0,1]);
% grid on
% l__ = legend('local descriptors only', 'SAP descriptors only', 'both descriptor types', 'Location', 'SouthEast')
% set(l__, 'Box', 'off')


%%%%% Visualization code
if (SingleClassVisualization)
    ROC = load('-ascii', [Folder, Object, '.txt'])
    ROCCurve(ROC, Object, Classifier, Description, 0);
    PRCurve(ROC, Object, Classifier, Description, 0);
    ROC
% else
%     ROC = load('-ascii', [Folder, Object, '.txt']);
%     Sum = zeros(size(ROC));
%     Count = 0;
%     for Object = Objects
%         Object = cell2mat(Object);
%         ROC = load('-ascii', [Folder, Object, '.txt']);
%         Sum = Sum + ROC;
%         Count = Count + 1;
%     end
%     Sum = Sum/Count;
%     ROCCurve(Sum, 'All', Classifier, Description);
% end
else
    Count = 1;
    Best = zeros(size(Objects));
    BestStats = zeros(size(Objects,2), 6);    % recall (true positive rate), false negative rate, precision; (1-3)=cross validation, (4-6)=test set
    ROC = cell(size(Objects));
    for Object = Objects
        Object = cell2mat(Object);
        ROC{Count} = load('-ascii', [Folder, Object, '.txt']);
        NearestDistance = 2;
        for i=1:size(ROC{Count}, 1)
            Distance = (sqrt((1-ROC{Count}(i, 4))^2 + (1-ROC{Count}(i, 3))^2));
            if (Distance < NearestDistance)
                Best(Count) = i;
                NearestDistance = Distance;
                BestStats(Count, 1) = ROC{Count}(i, 3);
                BestStats(Count, 2) = 1-ROC{Count}(i, 4);
                BestStats(Count, 3) = ROC{Count}(i, 5);
                BestStats(Count, 4) = ROC{Count}(i, 7);
                BestStats(Count, 5) = 1-ROC{Count}(i, 8);
                BestStats(Count, 6) = ROC{Count}(i, 9);
            end
        end
        Count = Count + 1;
    end
    BestStats(:,1:3)
    mBestStats = mean(BestStats, 1)
    performanceMatrix = [performanceMatrix; mBestStats(1:3)];
    
    MinIndex = min(Best);
    MaxIndex = max(Best);
    
    Rows = (size(ROC{1}, 1)-MaxIndex) - (1-MinIndex) + 3;
    ROCAll = zeros(Rows , size(ROC{1}, 2));
    ROCAll(1, 3) = 1;
    ROCAll(1, 7) = 1;
    ROCAll(Rows, 4) = 1;
    ROCAll(Rows, 5) = 1;
    ROCAll(Rows, 8) = 1;
    ROCAll(Rows, 9) = 1;
    Row = 2;
    for i=(1-MinIndex):(size(ROC{1}, 1)-MaxIndex)
        for o = 1:size(ROC,2)
            ROCAll(Row, 3) = ROCAll(Row, 3) + ROC{o}(Best(o)+i, 3);
            ROCAll(Row, 4) = ROCAll(Row, 4) + ROC{o}(Best(o)+i, 4);
            ROCAll(Row, 5) = ROCAll(Row, 5) + ROC{o}(Best(o)+i, 5);
            ROCAll(Row, 7) = ROCAll(Row, 7) + ROC{o}(Best(o)+i, 7);
            ROCAll(Row, 8) = ROCAll(Row, 8) + ROC{o}(Best(o)+i, 8);
            ROCAll(Row, 9) = ROCAll(Row, 9) + ROC{o}(Best(o)+i, 9);
        end
        ROCAll(Row, 3) = ROCAll(Row, 3)/size(ROC,2);
		ROCAll(Row, 4) = ROCAll(Row, 4)/size(ROC,2);
        if (ROCAll(Row, 5)/size(ROC,2) > ROCAll(Row-1, 5))
            ROCAll(Row, 5) = ROCAll(Row, 5)/size(ROC,2);
        else
            ROCAll(Row, 5) = ROCAll(Row-1, 5);
        end
        ROCAll(Row, 7) = ROCAll(Row, 7)/size(ROC,2);
		ROCAll(Row, 8) = ROCAll(Row, 8)/size(ROC,2);
        if (ROCAll(Row, 9)/size(ROC,2) > ROCAll(Row-1, 9))
            ROCAll(Row, 9) = ROCAll(Row, 9)/size(ROC,2);
        else
            ROCAll(Row, 9) = ROCAll(Row-1, 9);
        end
        Row = Row+1;
    end
    %if (polyOrder == 2 && linesX == 7)
        ROCCurve(ROCAll, 'All', Classifier, Description, 0);
        PRCurve(ROCAll, 'All', Classifier, Description, 0);
    %end
end

save('roc.mat', 'ROCAll');

% for curve fitting comparison
end
end
performanceMatrix
figure
set(gca, 'FontSize', 12)    %16
% with PCA pose normalization: multiClassResults = [[0.680864; 0.645370; 0.682716; 0.707099; 0.708796; 0.727778; 0.710648; 0.725309; 0.737654; 0.725463], [0.689660; 0.683025; 0.712963; 0.717130; 0.733951; 0.742130; 0.734568; 0.730864; 0.729938; 0.758025], [0.655864; 0.670679; 0.691204; 0.679012; 0.693519; 0.699537; 0.704475; 0.693210; 0.697994; 0.713426]];
% different random seeds: multiClassResults = [[0.699303; 0.680797; 0.732472; 0.706961; 0.728999; 0.758053; 0.722994; 0.749383; 0.763580], [0.702708; 0.697668; 0.704223; 0.730645; 0.728264; 0.708728; 0.740278; 0.727623; 0.732870], [0.703065; 0.686646; 0.663431; 0.716113; 0.703224; 0.739918; 0.728858; 0.722685; 0.726389]];
% different random seeds, from 1 to 10: 
%multiClassResults = [[0.65149; 0.699303; 0.680797; 0.732472; 0.706961; 0.728999; 0.758053; 0.722994; 0.749383; 0.763580], [0.692549; 0.702708; 0.697668; 0.704223; 0.730645; 0.728264; 0.708728; 0.740278; 0.727623; 0.771759], [0.673596; 0.703065; 0.686646; 0.663431; 0.716113; 0.703224; 0.739918; 0.728858; 0.722685; 0.726389]];
% with wrong computation: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.669012; 0.700366; 0.700336; 0.734221; 0.743747; 0.770047; 0.742512], [0.694159; 0.703811; 0.721603; 0.711525; 0.735642; 0.754630; 0.760031], [0.679838; 0.717847; 0.695060; 0.690087; 0.739984; 0.748803; 0.743544]]*100, 'LineWidth', 1.25)  %1.5
% without pose normalization: multiClassResults = [[0; 0.726235; 0.778549; 0.790278; 0.800926; 0.785340; 0.811728; 0.815123; 0.824228; 0.817901], [0; 0.735185; 0.748148; 0.770833; 0.775617; 0.782407; 0.772377; 0.774691; 0.794444; 0.782099], [0; 0.703549; 0.709568; 0.730247; 0.750926; 0.758488; 0.754784; 0.751852; 0.767593; 0.770525]];
% with roll compensation: 
multiClassResults = [[0; 0.755864; 0.760648; 0.791512; 0.789969; 0.789969; 0.785802; 0.760648; 0.755247; 0.781636], [0; 0.682099; 0.711420; 0.738735; 0.734414; 0.742130; 0.727469; 0.742747; 0.746759; 0.737191], [0; 0.658333; 0.692284; 0.699074; 0.680556; 0.706019; 0.707870; 0.706790; 0.699691; 0.702623]];


plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), multiClassResults(minLinesX:maxLinesX,1:3)]*100, 'LineWidth', 1.25)  %1.5
grid
ylim([60,100])
%ylim([65, 100])
xlabel('Number of cuts in x and y-Direction, n_x=n_y')
ylabel('Recall (%)')
l__ = legend('Order 2 Polynomial (one-against-all)', 'Order 4 Polynomial (one-against-all)', 'Order 6 Polynomial (one-against-all)', 'Order 2 Polynomial (multiple classes)', 'Order 4 Polynomial (multiple classes)', 'Order 6 Polynomial (multiple classes)', 'Location', 'West')
% l__ = legend('n_p = 2 (one-against-all)', 'n_p = 4 (one-against-all)', 'n_p = 6 (one-against-all)', 'n_p = 2 (multiple classes)', 'n_p = 4 (multiple classes)', 'n_p = 6 (multiple classes)', 'Location', 'West')
% set(l__, 'FontSize', 10)
set(l__, 'Box', 'off')

% for em comparison
% end
% performanceMatrix
% figure
% set(gca, 'FontSize', 12)    %16
% plot([[46, 75, 100, 150, 200, 250]', [46, 75, 100, 150, 200, 250]'], [performanceMatrix(1:6,1), [0.443298; 0.478948; 0.524801; 0.569029; 0.593274; 0.598888]]*100, 'LineWidth', 1.25)   %1.5
% grid
% xlim([50,250])
% ylim([35,85])
% xlabel('Number of Clusters')
% ylabel('Recall (%)')
% l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthEast')
% set(l__, 'Box', 'off')