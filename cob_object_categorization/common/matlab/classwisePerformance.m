close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% bow on ipa and ipa2, with 250 clusters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bow_ipa_recall = [  0.7875    0.2736        % binary | multiclass
                    0.7950    0.6215
                    0.8726    0.7292
                    0.8208    0.5732
                    0.7903    0.5444
                    0.8117    0.6513
                    0.9293    0.8384
                    0.9097    0.7694
                    0.8444    0.3889];
                
bow_ipa_precision = [   0.7821    0.3703        % binary | multiclass
                        0.8562    0.6035
                        0.8808    0.5936
                        0.8329    0.7432
                        0.8019    0.4612
                        0.8848    0.5869
                        0.9635    0.9463 
                        0.9317    0.6587
                        0.8096    0.4275];

figure
bar(100*bow_ipa_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*bow_ipa_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
bow_ipa2_recall = [ 0.8417    0.5889     % binary | multiclass
                    0.8028    0.1889
                    0.8278    0.4472
                    0.8806    0.5444
                    0.7389    0.3944
                    0.7667    0.4500
                    0.6917    0.5278
                    0.6750    0.0444
                    0.7639    0.5694
                    0.9528    0.1944
                    0.8556    0.2444
                    0.7972    0.4472
                    0.9528    0.7130
                    0.9500    0.7556];
                
bow_ipa2_precision = [  0.8842    0.5171        % binary | multiclass
                        0.7243    0.3696
                        0.8083    0.4204
                        0.8741    0.4083
                        0.7932    0.5299
                        0.8074    0.5978
                        0.7905    0.5723
                        0.6897    0.1778
                        0.8213    0.5601
                        0.7784    0.3286
                        0.7687    0.1702
                        0.8479    0.5458
                        0.8159    0.5532
                        0.9310    0.4866];

figure
bar(100*bow_ipa2_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*bow_ipa2_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sap7-7-2, pca, on ipa and ipa2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_pca_ipa_recall = [  0.9167    0.819444          % binary | multiclass
                        0.9597    0.888386
                        0.9347    0.656442
                        0.8736    0.62901
                        0.9208    0.7
                        0.8972    0.773676
                        0.9653    0.998575
                        0.9236    0.818056
                        0.8486    0.538889];
                
sap_pca_ipa_precision = [   0.9145    0.783533        % binary | multiclass
                            0.9671    0.836648
                            0.9421    0.575269
                            0.8674    0.703588
                            0.9006    0.765957
                            0.9625    0.695527
                            0.9988    0.992918
                            0.9015    0.875186
                            0.8747    0.583459];

figure
bar(100*sap_pca_ipa_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_pca_ipa_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_pca_ipa2_recall = [ 0.9556	0.900000     % binary | multiclass
                        0.9111	0.761111
                        0.8639	0.694444
                        0.8556	0.691667
                        0.8222	0.744444
                        0.9750	0.933333
                        0.9361	0.872222
                        0.8778	0.372222
                        0.8806	0.822222
                        0.9806	0.905556
                        0.9556	0.488889
                        0.9639	0.911111
                        0.9731	0.541667
                        0.8750	0.613889];

sap_pca_ipa2_precision = [  0.9498	0.818182        % binary | multiclass
                            0.9218	0.886731
                            0.8844	0.598086
                            0.8411	0.682192
                            0.8889	0.696104
                            0.9692	0.854962
                            0.9556	0.839572
                            0.8327	0.635071
                            0.9346	0.766839
                            0.9494	0.505426
                            0.8877	0.451282
                            0.9577	0.784689
                            0.9516	0.776892
                            0.9256	0.697161];

figure
bar(100*sap_pca_ipa2_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_pca_ipa2_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sap7-7-2, nonorm, on ipa and ipa2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_nonorm_ipa_recall = [   0.8597    0.6361          % binary | multiclass
                            0.9778    0.9639
                            0.9667    0.7306
                            0.9306    0.6500
                            0.9375    0.8333
                            0.9000    0.8847
                            0.9014    0.9875
                            0.9875    0.8264
                            0.9778    0.7931];



                
sap_nonorm_ipa_precision = [0.9315    0.9178        % binary | multiclass
                            0.9746    0.8213
                            0.9521    0.6196
                            0.9124    0.8269
                            0.9399    0.7417
                            0.9739    0.8209
                            0.9980    0.9861
                            0.9687    0.8287
                            0.9728    0.8192];

figure
bar(100*sap_nonorm_ipa_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_nonorm_ipa_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_nonorm_ipa2_recall = [  0.9778    0.8861     % binary | multiclass
                            0.9389    0.8639
                            0.8944    0.5556
                            0.8833    0.8389
                            0.9583    0.8583
                            0.9833    0.9472
                            0.9778    0.8694
                            0.9306    0.5333
                            0.9889    0.9778
                            0.9944    0.9111
                            0.9556    0.3222
                            0.9639    0.9528
                            0.8954    0.7259
                            0.9361    0.6639];

sap_nonorm_ipa2_precision = [   0.9787    0.9410        % binary | multiclass
                                0.9320    0.8383
                                0.8928    0.5236
                                0.9622    0.8320
                                0.9601    0.8026
                                0.9659    0.9045
                                0.9805    0.9572
                                0.9145    0.5598
                                0.9940    0.9387
                                0.9606    0.5910
                                0.8847    0.5202
                                0.9720    0.8932
                                0.9260    0.7664
                                0.9267    0.7636];

figure
bar(100*sap_nonorm_ipa2_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_nonorm_ipa2_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sap7-7-2, rollnorm, on ipa and ipa2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_rollnorm_ipa_recall = [ 0.8264    0.8264          % binary | multiclass
                            0.9722    0.9486
                            0.9653    0.6806
                            0.8847    0.6861
                            0.8708    0.7306
                            0.8986    0.8736
                            1.0000    0.9764
                            0.9931    0.9125
                            0.9111    0.4375];



                
sap_rollnorm_ipa_precision = [  0.9154    0.8737        % binary | multiclass
                                0.9827    0.8999
                                0.9484    0.5819
                                0.9102    0.8359
                                0.8892    0.6600
                                0.9695    0.7775
                                0.9986    0.9683
                                0.9686    0.8061
                                0.8995    0.6848];

figure
bar(100*sap_rollnorm_ipa_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_rollnorm_ipa_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:9, 'XTickLabel', ' Ball| Book| Bottle| Coffeepot| Plush Toy| Cup| Flowerpot| Drink Carton| Toy Car', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sap_rollnorm_ipa2_recall = [0.9694    0.8917        % binary | multiclass
                            0.9139    0.8472
                            0.8639    0.6500
                            0.8750    0.7556
                            0.9667    0.8694
                            0.9917    0.9444
                            0.9472    0.8278
                            0.9306    0.5750
                            0.9833    0.9389
                            0.9750    0.8667
                            0.9222    0.5639
                            0.9667    0.9111
                            0.9463    0.4306
                            0.9111    0.7139];

sap_rollnorm_ipa2_precision = [ 0.9794    0.9497        % binary | multiclass
                                0.9250    0.8971
                                0.8835    0.5367
                                0.9599    0.8293
                                0.9569    0.8482
                                0.9736    0.9067
                                0.9570    0.8843
                                0.9099    0.6053
                                0.9607    0.8264
                                0.9683    0.4664
                                0.8799    0.3845
                                0.9604    0.8723
                                0.9211    0.7776
                                0.9309    0.8159];

figure
bar(100*sap_rollnorm_ipa2_recall, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Recall (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)

figure
bar(100*sap_rollnorm_ipa2_precision, 'LineWidth', 1.25, 'BarWidth', 1.0)
grid on
set(gca, 'FontSize', 12)
set(gca,'XTick',1:14, 'XTickLabel', ' Binder| Book| Bottle| Can| Coffeepot| Cup| Dishes| Dish Liquid| Mouse| Pen| Scissors| Screen| Silverware| Drink Carton', 'FontSize', 10);
rotateticklabel(gca, 335);
%xlabel('Object Class')
ylabel('Precision (%)')
ylim([0,100])
l__ = legend('One-against-all', 'Multiple classes', 'Location', 'SouthWest');
%set(l__, 'Box', 'off')
set(l__, 'FontSize', 10)







