function confusionMatrix()
    %close all
    clear all
    
    confusionMatrixIPA2;
end

function confusionMatrixIPA2()
    % IPA-2
    objects = {'binder', 'book', 'bottle', 'can', 'coffeepot', 'cup', 'dishes', 'dishliquids', 'mouse', 'pen', 'scissors', 'screens', 'silverware', 'tetrapaks'}
    stats = zeros(size(objects,2));     %1.index = true class,  2.index = recognized class
    viewWiseErrors = zeros(size(objects,2), 36);
    folds = 10;

    % filename = '../files/IPA2Data/exp 13 - bow cluster/EM250 - normalization - oldmask/Classifier/Statistics/individualResults.txt';
    % filename = '../files/IPA2Data/exp 1 - PCA3CF/PCA3CF7-7-2/Classifier/Statistics/individualResults.txt';
    % filename = '../files/IPA2Data/exp 2 - PCA3CF no pose normalization/PCA3CF7-7-2/Classifier/Statistics/individualResults.txt';
    % filename = '../files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/Statistics/individualResults.txt';
    % filename = '../files/IPA2Data/exp 11 - double sap/PCA3CF7-7-2+5-5-4/Classifier/Statistics/individualResults.txt';
    filename = '../files/IPA2Data/exp 12 - combo/EM60PCA3CF7-7-2 - no normalization - oldmask/Classifier/Statistics/individualResults.txt';
    
    fid = fopen(filename, 'r');

    for object=objects
        name = fscanf(fid, '%s', 1);
        if (strcmp(name, object)==0)
            disp('name and object are not the same.')
            break;
        end

        folds_ = folds;
        if (strcmp(object, 'silverware')==1)
            folds_ = 3*folds;
        end
        for fold = 1:folds_
            fscanf(fid, '%d', 1);
            for i=1:36
                recognized = fscanf(fid, '%s', 1);
                stats(nameToNum(object, objects), nameToNum(recognized, objects)) = stats(nameToNum(object, objects), nameToNum(recognized, objects)) + 1;
                if (strcmp(object, recognized)==0)
                    viewWiseErrors(nameToNum(object, objects), i) = viewWiseErrors(nameToNum(object, objects), i) + 1;
                end
            end
        end
    end
    
    fclose(fid);

    % make view-wise error plot
    viewWiseErrors(13,:) = viewWiseErrors(13,:)/3;
    viewWiseError(viewWiseErrors);
 
    % make confusion matrix
    for i=1:size(stats, 1)
        s = 0;
        for j=1:size(stats, 2)
            s = s + stats(i,j);
        end
        for j=1:size(stats, 2)
            stats(i,j) = stats(i,j)/s;
        end
    end
    
    figure
    set(gca, 'FontSize', 12)
    stats = [[stats, ones(size(stats,1),1)]; zeros(1, size(stats, 2)+1)];
    pcolor(stats)
    %colormap(1-gray)
    caxis([0,1])
    axis square
    title('prediction')
    ylabel('ground truth')
    %set(gca,'YDir','reverse')
    set(gca,'XTick',-0.2:12.8, 'XTickLabel', '             binder|             book|             bottle|             can|             coffeepot|             cup|             dishes|             dishliquid|             mouse|             pen|             scissors|             screen|             silverware|             drink carton', 'FontSize', 10);
    ROTATETICKLABEL(gca, 330);
    set(gca,'YTick',1.5:1:14.5, 'YTickLabel', 'binder|book|bottle|can|coffeepot|cup|dishes|dishliquid|mouse|pen|scissors|screen|silverware|drink carton', 'FontSize', 10);
    colorbar
    %set(gcf, 'Position',[550 500 560 416])
end


function confusionMatrixIPA1()
    % IPA-1
    objects = {'ball', 'book', 'bottle', 'coffeepot', 'cuddly', 'cup', 'flowerpot', 'tetrapack', 'vehicle'}
    stats = zeros(size(objects,2));     %1.index = true class,  2.index = recognized class
    viewWiseErrors = zeros(size(objects,2), 72);
    folds = 10;

    % filename = '../files/IPAData/Exp6 - EM/EM250/Classifier/Statistics/individualResults_72.txt';
    % filename = '../files/IPAData/Exp7 - PCA3CF/PCA3CF7-7-2/Classifier/Statistics/individualResults_72.txt';
    % filename = '../files/IPAData/Exp13 - PCA3CF-nonorm/PCA3CF7-7-2/Classifier/Statistics/individualResults_72.txt';
    filename = '../files/IPAData/Exp15 - PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/Statistics/individualResults_72.txt';
    fid = fopen(filename, 'r');

    for object=objects
        name = fscanf(fid, '%s', 1);
        if (strcmp(name, object)==0)
            disp('name and object are not the same.')
            break;
        end

        for fold = 1:folds
            temp = fscanf(fid, '%d', 1);
            for i=1:72
                recognized = fscanf(fid, '%s', 1);
                {i, recognized}
                if (nameToNum(recognized, objects) ~= size(objects,2)+1)
                    % recognized is a valid object (some methods do not deliver an output for every view --> we have to deal with missing views)
                    stats(nameToNum(object, objects), nameToNum(recognized, objects)) = stats(nameToNum(object, objects), nameToNum(recognized, objects)) + 1;
                    if (strcmp(object, recognized)==0)
                        viewWiseErrors(nameToNum(object, objects), i) = viewWiseErrors(nameToNum(object, objects), i) + 1;
                    end
                end
            end
        end
    end
    
    fclose(fid);

    % make view-wise error plot
    viewWiseError(viewWiseErrors);
    
    % make confusion matrix
    for i=1:size(stats, 1)
        s = 0;
        for j=1:size(stats, 2)
            s = s + stats(i,j);
        end
        for j=1:size(stats, 2)
            stats(i,j) = stats(i,j)/s;
        end
    end
    
    figure
    set(gca, 'FontSize', 12)
    stats = [[stats, ones(size(stats,1),1)]; zeros(1, size(stats, 2)+1)];
    pcolor(stats)
    %colormap(1-gray)
    caxis([0,1])
    axis square
    title('prediction')
    ylabel('ground truth')
    %set(gca,'YDir','reverse')
    set(gca,'XTick',1.5:9.5, 'XTickLabel', ' ball| book| bottle| coffeepot| cuddly| cup| flowerpot| drink carton| vehicle', 'FontSize', 10);
    ROTATETICKLABEL(gca, 330);
    set(gca,'YTick',1.5:1:9.5, 'YTickLabel', 'ball|book|bottle|coffeepot|plush toy|cup|flowerpot|drink carton|vehicle', 'FontSize', 10);
    colorbar
end


function num = nameToNum(name, objects)
    for num = 1:size(objects,2)
        if (strcmp(name, objects(num))==1)
            break;
        end
        if (num == size(objects,2))
            disp('object not found in the list.')
            num = size(objects,2)+1;
        end
    end
end