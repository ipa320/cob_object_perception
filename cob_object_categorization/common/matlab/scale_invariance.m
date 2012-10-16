% with pose normalization on IPA-2:
pointpercentage_sap772_pcanorm = [1, 0.5, 0.25, 0.1, 0.0625, 0.04];
distance_sap772_pcanorm = [1, 1.4, 2, 3.2, 4, 5];
% performance on PCA3CF7-7-2
performance_sap772_pcanorm = [0.732341, 0.732, 0.731, 0.695, 0.679, 0.658];

% without pose normalization on IPA-2:
pointpercentage_sap772_nonorm = [1, 0.5, 0.25, 0.1, 0.0625, 0.04];
distance_sap772_nonorm = [1, 1.4, 2, 3.2, 4, 5];
% performance on PCA3CF7-7-2 nonorm with IPA-2
performance_sap772_nonorm = [0.779034, 0.782, 0.771, 0.749, 0.748, 0.736];

% with roll pose normalization on IPA-2:
pointpercentage_sap772_rollnorm = [1, 0.5, 0.25, 0.1, 0.0625, 0.04];
distance_sap772_rollnorm = [1, 1.4, 2, 3.2, 4, 5];
% performance on PCA3CF7-7-2
performance_sap772_rollnorm = [0.769643, 0.766, 0.764, 0.754, 0.757, 0.755];

% with vfh on IPA-2:
pointpercentage_vfh = [1, 0.25, 0.1, 0.04];
distance_vfh = [1, 2, 3.2, 5];
% performance on vfh
performance_vfh = [0.684127, 0.701455, 0.643254, 0.560516];

distance = distance_vfh;
performance772 = performance_vfh;
% distance = distance_sap772_rollnorm;
% performance772 = performance_sap772_rollnorm;

figure(1)
set(gca, 'FontSize', 12)
plot(distance, [performance772']*100, 'LineWidth', 1.25)
grid
xlim([1,5])
ylim([0, 100])
xlabel('Distance Factor of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)


figure(2)
set(gca, 'FontSize', 12)
plot(distance, [performance_sap772_pcanorm', performance_sap772_rollnorm', performance_sap772_nonorm']*100, 'LineWidth', 1.25)
grid
xlim([1,5])
ylim([0, 100])
xlabel('Distance Factor  of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2 PCA normalized', 'SAP-7-7-2 roll compensation', 'SAP-7-7-2 not normalized', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)