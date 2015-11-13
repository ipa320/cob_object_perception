clc
%close all
clear all

SingleClassVisualization = 0;
Object = 'ball';

%%%%% --------- IPA2 Database
Objects = {'binder', 'book', 'bottle', 'can', 'coffeepot', 'cup', 'dishes', 'dishliquids', 'mouse', 'pen', 'scissors', 'screens', 'silverware', 'tetrapaks'}

performanceMatrix = [];

% Folder = '../files/IPAData/Exp10 - EMPCA3CF on Surf64/Classifier/Statistics/ROCCurve_IPA_Surf64_EM200PCA3CF6-6-2_RTC_';
% Description = 'IPA, Surf descriptor, 200 cluster histogram + 3 PCA + Curve fitting 6-6-2';
% Classifier = 'Random Trees Classifier';

% % for em comparison
% for em=[50, 100, 150, 200, 250]
% Folder = '';
% % if (em < 100)
% %     Folder = ['../files/IPA2Data/exp 13 - bow cluster/EM0', num2str(em), ' - normalization - oldmask/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_EM0', num2str(em), '_RTC_'];
% % else
%     Folder = ['../files/IPA2Data/exp 13 - bow cluster/EM', num2str(em), ' - normalization - oldmask/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_EM', num2str(em), '_RTC_'];
% % end
% Description = 'IPA2, Surf + 3DEnv histogram descriptor';
% Classifier = 'Random Trees Classifier';

% for curve fitting comparison
minLinesX = 2;
maxLinesX = 10;
for polyOrder=2:2:6
for linesX=minLinesX:maxLinesX
linesY=linesX;
% Folder = ['../files/IPA2Data/exp 1 - PCA3CF/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
% Folder = ['../files/IPA2Data/exp 2 - PCA3CF no pose normalization/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
% Folder = ['../files/IPA2Data/exp 6 - single descriptors/CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
% Folder = ['../files/IPA2Data/exp 6 - single descriptors - nonorm/CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
Folder = ['../files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_PCA3CF', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder), '_RTC_'];
Description = ['IPA2, 3 PCA + Curve fitting ', num2str(linesX), '-', num2str(linesY), '-', num2str(polyOrder)];
Classifier = 'Random Trees Classifier';
polyOrder
linesX

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

% Folder = '../files/IPA2Data/exp 13 - bow cluster/EM250 - normalization - oldmask/Classifier/Statistics/ROCCurve_IPA2_Surf64Dev2_EM250_RTC_';
% Description = ''; %Description = 'IPA, bag of local features descriptor, 250 cluster histogram';
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

% for curve fitting comparison (sloppy masks)
end
end
performanceMatrix
figure
set(gca, 'FontSize', 12)    %16
% multiclass performance: pose normalized point clouds
% oldmask: 
multiClassResults = [[0.667196; 0.683598; 0.706085; 0.716865; 0.722553; 0.714352; 0.732341; 0.732143; 0.726058; 0.735648], [0.652315; 0.660714; 0.680754; 0.689286; 0.681019; 0.687169; 0.687765; 0.692262; 0.692593; 0.688161], [0.641601; 0.648810; 0.651389; 0.667394; 0.660648; 0.662103; 0.655489; 0.668056; 0.666865; 0.668783]];      % multiclass performance: pose normalized point clouds
% comparison to descriptor without PCA3 part:
%curve22 = [0.9141, 0.9213, 0.9198, 0.9172, 0.9110, 0.9161, 0.9222, 0.9191, 0.9183; 0.683598, 0.706085, 0.716865, 0.722553, 0.714352, 0.732341, 0.732143, 0.726058, 0.735648]';
%plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [curve22(:,1), performanceMatrix(1:(maxLinesX-minLinesX+1),1), curve22(:,2), [0.530952; 0.577116; 0.590873; 0.587103; 0.607738; 0.639418; 0.612235; 0.627447; 0.614153]]*100, 'LineWidth', 1.25)      % multiclass performance: pose normalized point clouds

% goodmask: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.694841; 0.722553; 0.729894; 0.721032; 0.72328; 0.740939; 0.733399; 0.740608; 0.737831], [0.678373; 0.686706; 0.700066; 0.692857; 0.697421; 0.681283; 0.700926; 0.686905; 0.69332], [0.662103; 0.646892; 0.677579; 0.657937; 0.662103; 0.663757; 0.655886; 0.676124; 0.660516]]*100, 'LineWidth', 1.25)      % multiclass performance: pose normalized point clouds
% curve22 = [performanceMatrix(1:(maxLinesX-minLinesX+1),1), [0.694841; 0.722553; 0.729894; 0.721032; 0.72328; 0.740939; 0.733399; 0.740608; 0.737831]]
%[0.659458; 0.66455; 0.657937; 0.662831; 0.662831; 0.664616; 0.67123; 0.680489; 0.668585]]
% without PCA3 component:
%curve22=[0.913558214285714,0.694841000000000;0.922420642857143,0.722553000000000;0.916666714285714,0.729894000000000;0.908465571428571,0.721032000000000;0.920634785714286,0.723280000000000;0.920436500000000,0.740939000000000;0.917063571428571,0.733399000000000;0.916799000000000,0.740608000000000;0.923941857142857,0.737831000000000];
% plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [curve22(:,1), performanceMatrix(1:(maxLinesX-minLinesX+1),1), curve22(:,2), [0.492328; 0.522288; 0.564616; 0.574405; 0.572619; 0.577778; 0.605489; 0.609392; 0.607209]]*100, 'LineWidth', 1.25)      % multiclass performance: pose normalized point clouds
% with wrong computation: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.694114; 0.726190; 0.735780; 0.733664; 0.744577; 0.742659; 0.744974; 0.745701; 0.748347], [0.677976; 0.702976; 0.700926; 0.710582; 0.704630; 0.712765; 0.710648; 0.713426; 0.713690], [0.648016; 0.662831; 0.675860; 0.668188; 0.670635; 0.667328; 0.674140; 0.671098; 0.675066]]*100, 'LineWidth', 1.25)      % multiclass performance: pose normalized point clouds

% multiclass performance: unnormalized point clouds
% oldmask: multiClassResults = [[0.730291; 0.710913; 0.757738; 0.739153; 0.765278; 0.769643; 0.779034; 0.766468; 0.762632; 0.767460], [0.723479; 0.709458; 0.749272; 0.745899; 0.761971; 0.754828; 0.767923; 0.755357; 0.778505; 0.744643], [0.724669; 0.678373; 0.729365; 0.702976; 0.736177; 0.717328; 0.743386; 0.717460; 0.727183; 0.740079]];
% multiClassResults = [[0.730291; 0.710913; 0.757738; 0.739153; 0.765278; 0.769643; 0.779034; 0.766468; 0.762632; 0.767460], [0.723479; 0.709458; 0.749272; 0.735899; 0.761971; 0.754828; 0.767923; 0.755357; 0.758505; 0.744643], [0.724669; 0.678373; 0.729365; 0.702976; 0.736177; 0.717328; 0.743386; 0.717460; 0.727183; 0.740079]];
% curve22 = [[0.9416; 0.9499; 0.9537; 0.9470; 0.9473; 0.9485; 0.9479; 0.9534; 0.9486], [0.710913; 0.757738; 0.739153; 0.765278; 0.769643; 0.779034; 0.766468; 0.762632; 0.767460]];
% plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [curve22(:,1), performanceMatrix(1:(maxLinesX-minLinesX+1),1), curve22(:,2), [0.578571; 0.633003; 0.620238; 0.670833; 0.687368; 0.695966; 0.701521; 0.691799; 0.702579]]*100, 'LineWidth', 1.25)      % multiclass performance: pose normalized point clouds

% goodmask: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.712765; 0.758267; 0.735847; 0.762500; 0.779101; 0.776587; 0.771627; 0.769048; 0.762831], [0.708929; 0.753638; 0.743188; 0.759854; 0.760714; 0.769974; 0.752183; 0.778505; 0.743915], [0.688757; 0.732209; 0.701918; 0.738029; 0.714484; 0.741733; 0.714484; 0.733730; 0.738889]]*100, 'LineWidth', 1.25)      % multiclass performance: unnormalized point clouds
% goodmask: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.710913; 0.757738; 0.739153; 0.765278; 0.769643; 0.779034; 0.766468; 0.762632; 0.767460], [0.708929; 0.753638; 0.743188; 0.759854; 0.760714; 0.769974; 0.752183; 0.765505; 0.743915], [0.688757; 0.732209; 0.701918; 0.738029; 0.714484; 0.741733; 0.714484; 0.733730; 0.738889]]*100, 'LineWidth', 1.25)      % multiclass performance: unnormalized point clouds
% with wrong computation: plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), [0.714021; 0.771958; 0.741402; 0.775397; 0.779696; 0.787632; 0.762368; 0.759392; 0.765013], [0.727381; 0.766270; 0.756415; 0.779365; 0.773347; 0.776190; 0.777447; 0.779762; 0.750000], [0.711442; 0.739484; 0.730820; 0.750860; 0.738161; 0.750529; 0.745966; 0.749537; 0.744312]]*100, 'LineWidth', 1.25)      % multiclass performance: unnormalized point clouds
% comparing to CFx-x-y : plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [[0.9416; 0.9499; 0.9537; 0.9470; 0.9473; 0.9485; 0.9479; 0.9534; 0.9486], performanceMatrix(1:(maxLinesX-minLinesX+1),1), [0.712765; 0.758267; 0.741402; 0.762500; 0.779101; 0.776587; 0.771627; 0.769048; 0.762831], [0.578571; 0.633003; 0.620238; 0.670833; 0.687368; 0.695966; 0.701521; 0.691799; 0.702579]]*100, 'LineWidth', 1.25)      % multiclass performance: unnormalized point clouds      

% multiclass performance: roll normalized point clouds
% oldmask: 
multiClassResults = [[0, 0.713558, 0.741667, 0.727579, 0.757870, 0.749074, 0.769643, 0.763294, 0.753307, 0.766336]; [0, 0.690079, 0.735317, 0.698479, 0.742659, 0.742196, 0.738492, 0.753042, 0.728968, 0.739947]; [0, 0.661376, 0.692989, 0.682011, 0.712566, 0.713360, 0.712897, 0.712698, 0.705489, 0.711442]]';      % multiclass performance: pose normalized point clouds


plot([(minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)', (minLinesX:maxLinesX)'], [performanceMatrix(1:(maxLinesX-minLinesX+1),1), performanceMatrix(maxLinesX-minLinesX+1+1:2*(maxLinesX-minLinesX+1),1), performanceMatrix(2*(maxLinesX-minLinesX+1)+1:3*(maxLinesX-minLinesX+1),1), multiClassResults(minLinesX:maxLinesX, 1:3)]*100, 'LineWidth', 1.25)      % multiclass performance: unnormalized point clouds
grid
xlabel('Number of cuts in x and y-Direction, n_x=n_y')
ylabel('Recall (%)')
 ylim([60,100])
l__ = legend('Order 2 Polynomial (one-against-all)', 'Order 4 Polynomial (one-against-all)', 'Order 6 Polynomial (one-against-all)', 'Order 2 Polynomial (multiple classes)', 'Order 4 Polynomial (multiple classes)', 'Order 6 Polynomial (multiple classes)', 'Location', 'West');
% l__ = legend('with Eigenvalues (one-against-all)', 'without Eigenvalues (one-against-all)', 'with Eigenvalues (multiple classes)', 'without Eigenvalues (multiple classes)', 'Location', 'SouthEast');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)

% % for em comparison (sloppy masks)
% end
% performanceMatrix
% figure
% set(gca, 'FontSize', 12)
% plot([[50, 100, 150, 200, 250]', [50, 100, 150, 200, 250]'], [performanceMatrix(1:5,1), [0.386971; 0.394974; 0.411574; 0.418452; 0.436442]]*100, 'LineWidth', 1.25)
% grid
% xlim([50,250])
% ylim([35 85])
% xlabel('Number of Clusters')
% ylabel('Recall (%)')
% l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthEast')
% set(l__, 'Box', 'off')