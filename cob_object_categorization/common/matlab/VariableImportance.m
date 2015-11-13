%for i=1:19
    Folder = '../files/IPAData/Classifier/Statistics/VarImportance_IPA_Surf64Dev2_EM184PCA3CF12FS6_RTC_';
    Object = 'ball';

    VarImportance = load('-ascii', [Folder, Object, '.txt'])
    bar((1:size(VarImportance, 2)), VarImportance(6:8,:)', 'grouped')
    hold on
%end