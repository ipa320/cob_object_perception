function viewWiseError(input)
    % start from confusionMatrix.m

    viewWiseErrors = input;

    figure('Position',[100 500 1120 416])%, 'PaperOrientation', 'landscape')
    set(gca, 'FontSize', 12)
    viewWiseErrors = [[viewWiseErrors, ones(size(viewWiseErrors,1),1)]; zeros(1, size(viewWiseErrors, 2)+1)];
    pcolor(viewWiseErrors)
    %colormap(1-gray)
    caxis([0,10])
    %axis square
    %title('prediction')
    %ylabel('ground truth')
    xlabel('Pan Angle (°)')
    if (size(input,1) == 9)
        % IPA-1 dataset
        set(gca,'XTick',[1.5 19.5 37.5 55.5 72.5], 'XTickLabel', '0|90|180|270|355', 'FontSize', 12);
        set(gca,'YTick',1.5:1:9.5, 'YTickLabel', 'ball|book|bottle|coffeepot|plush toy|cup|flowerpot|drink carton|vehicle', 'FontSize', 10);
        set(gca,'YDir','reverse')
    elseif (size(input,1) == 14)
        % IPA-2 dataset
        set(gca,'XTick',[1.5 10.5 19.5 28.5 36.5], 'XTickLabel', '0|90|180|270|350', 'FontSize', 12);
        set(gca,'YTick',1.5:1:14.5, 'YTickLabel', 'binder|book|bottle|can|coffeepot|cup|dishes|dishliquid|mouse|pen|scissors|screen|silverware|drink carton', 'FontSize', 10);
        set(gca,'YDir','reverse')
    else
        disp('Error in viewWiseError: wrong size of input array.')
    end
    colorbar
end


function viewWiseErrorSAPPCA_772_ipa2()
    viewWiseErrors_sappca_772_ipa2 = [  0	0	0	0	1	0	0	2	3	5	7	2	3	1	1	0	0	0	0	0	0	0	0	0	0	1	2	7	5	1	1	2	1	0	0	0
                                    0	0	1	1	1	2	1	3	8	4	6	4	2	3	3	1	0	0	0	0	1	1	1	2	3	2	4	6	5	5	3	2	1	2	1	0
                                    2	4	3	3	2	3	2	2	4	3	4	4	1	2	3	2	4	2	2	2	5	3	4	2	1	3	4	3	4	3	4	2	2	4	5	2
                                    4	4	3	3	3	3	3	3	3	3	3	3	3	3	3	3	3	3	2	3	3	3	2	3	3	3	3	4	3	3	3	4	3	4	3	5
                                    2	2	2	2	2	2	2	3	2	2	2	3	4	4	4	3	3	2	2	3	3	2	2	2	2	3	2	2	2	2	2	2	3	4	2	2
                                    1	1	1	2	0	0	1	1	1	1	1	1	0	0	0	1	0	0	0	1	0	0	0	1	0	1	1	1	1	1	2	1	0	0	1	1
                                    0	0	0	0	0	0	2	5	5	3	4	5	2	1	0	0	1	0	1	0	0	0	0	1	1	1	3	5	6	4	1	0	0	0	0	0
                                    5	6	7	6	5	5	2	4	5	7	5	5	5	6	7	7	7	7	8	6	7	6	7	4	4	3	7	5	6	2	7	3	6	7	6	8
                                    3	1	3	2	0	0	2	1	2	3	4	1	1	3	0	0	2	2	4	4	2	2	2	0	0	1	1	1	2	2	3	2	2	1	2	4
                                    0	1	1	0	0	0	1	0	1	0	0	0	1	0	0	1	1	1	1	1	2	1	1	1	2	1	1	1	1	2	2	1	0	2	1	1
                                    2	0	0	2	5	7	5	8	10	10	10	9	3	4	3	3	2	2	0	0	0	1	3	5	8	10	10	10	10	9	5	2	2	2	2	2
                                    1	0	0	0	0	0	0	2	1	4	1	0	0	0	1	1	0	1	0	1	1	1	1	1	0	1	1	2	0	5	1	0	1	0	1	0
                                    11	12	17	17	18	8	10	12	8	11	11	8	14	16	15	16	18	23	24	22	20	18	17	16	9	8	10	8	5	11	11	16	15	19	13	9
                                    9	7	5	2	1	1	0	0	7	7	7	2	2	3	3	4	4	4	8	8	2	5	2	1	2	4	7	9	7	0	2	0	0	2	4	3
    ];
    viewWiseErrors_sappca_772_ipa2(13,:) = viewWiseErrors_sappca_772_ipa2(13,:)/3;
    
    viewWiseError(viewWiseErrors_sappca_772_ipa2);
end