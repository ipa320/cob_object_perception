% with pose normalization on IPA-2:
viewNumbers_sap772_pcanorm = [36, 24, 18, 16, 12, 8, 6, 4];
angles_sap772_pcanorm = [0, 180./viewNumbers_sap772_pcanorm(2:end)];
% performance on PCA3CF7-7-2
performance_sap772_pcanorm = [0.732341, 0.736045, 0.723082, 0.695370, 0.725331, 0.667460, 0.683532, 0.597619];
%performance772 = [0.779034, 0.766997, 0.769709, 0.744577, 0.733069, 0.718585, 0.710317, 0.617262];
% performance on PCA3CF2-2-2
performance_sap222_pcanorm = [0.683598, 0.691005, 0.681548, 0.68201, 0.682672, 0.654960, 0.657804, 0.597884];
% performance on PCA3CF10-10-2
performance_sap10102_pcanorm = [0.735648, 0.735516, 0.721362, 0.696362, 0.726190, 0.668783, 0.680820, 0.578042];


% without pose normalization on IPA-2:
viewNumbers_sap772_nonorm = [36, 24, 18, 16, 12, 8, 6, 4];
angles_sap772_nonorm = [0, 180./viewNumbers_sap772_nonorm(2:end)];
% performance on PCA3CF7-7-2 nonorm with IPA-2
performance_sap772_nonorm = [0.779034, 0.766997, 0.769709, 0.733069, 0.744577, 0.710317, 0.718585, 0.617262];
%performance772 = [0.779034, 0.766997, 0.769709, 0.744577, 0.733069, 0.718585, 0.710317, 0.617262];
% performance on PCA3CF2-2-2 nonorm with IPA-2
performance_sap222_nonorm = [0.710913, 0.694312, 0.694180, 0.699140, 0.692791, 0.656481, 0.669180, 0.563955];
% performance on PCA3CF10-10-2 nonorm with IPA-2
performance_sap10102_nonorm = [0.767460, 0.735450, 0.742262, 0.719180, 0.733069, 0.684325, 0.715741, 0.606614];

% with roll pose normalization on IPA-2:
viewNumbers_sap772_rollnorm = [36, 24, 18, 16, 12, 8, 6, 4];
angles_sap772_rollnorm = [0, 180./viewNumbers_sap772_rollnorm(2:end)];
% performance on PCA3CF7-7-2
performance_sap772_rollnorm = [0.769643, 0.758796, 0.751521, 0.741336, 0.730820, 0.684722, 0.682341, 0.589352];
% performance on PCA3CF2-2-2
performance_sap222_rollnorm = [0.713558, 0.705886, 0.697884, 0.713095, 0.699405, 0.671429, 0.656415, 0.574471];
% performance on PCA3CF10-10-2
performance_sap10102_rollnorm = [0.766336, 0.751124, 0.748743, 0.731812, 0.741468, 0.684458, 0.698082, 0.582341];

% with vfh on IPA-2:
viewNumbers_vfh = [36, 24, 18, 16, 12, 8, 6, 4];
angles_vfh = [0, 180./viewNumbers_vfh(2:end)];
% performance on vfh
performance_vfh = [0.684127, 0.649802, 0.658333, 0.608929, 0.640873, 0.588823, 0.583664, 0.532143];


viewNumbers = viewNumbers_sap772_nonorm;
angles = angles_sap772_nonorm;
performance772 = performance_sap772_nonorm;
performance222 = performance_sap222_nonorm;
performance10102 = performance_sap10102_nonorm;


figure(1)
set(gca, 'FontSize', 12)
%plot(viewNumbers, [performance222', performance772', performance10102']*100, 'LineWidth', 1.25)
plot(angles, [performance772', performance222', performance10102']*100, 'LineWidth', 1.25)
grid
%xlim([4,36])
xlim([0,45])
ylim([0, 100])
%xlabel('Number of Training Views per Object')
xlabel('Pan Angle of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2', 'SAP-2-2-2', 'SAP-10-10-2', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)


figure(2)
set(gca, 'FontSize', 12)
%plot(viewNumbers, [performance222', performance772', performance10102']*100, 'LineWidth', 1.25)
plot(angles, [performance_sap772_pcanorm', performance_sap772_rollnorm', performance_sap772_nonorm']*100, 'LineWidth', 1.25)
grid
%xlim([4,36])
xlim([0,45])
ylim([0, 100])
%xlabel('Number of Training Views per Object')
xlabel('Pan Angle of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2 PCA normalized', 'SAP-7-7-2 roll compensation', 'SAP-7-7-2 not normalized', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)