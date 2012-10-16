tilt_sap772_nonorm = [
0   0.779034;
5   0.753704;
10  0.745370;
15  0.729894;
20  0.724934;
25  0.699471;
30  0.693122;
35  0.675728;
40  0.635053;
%45  0.520899;
];

tilt_sap772_pcanorm = [
0   0.742593;
5   0.736706;
10  0.739286;
15  0.738624;
20  0.729894;
25  0.732870;
30  0.722553;
35  0.714021;
40  0.670767
];

tilt_sap772_rollnorm = [
0   0.769643;
5   0.760163;
10  0.750220;
15  0.734987;
20  0.727800;
25  0.723104;
30  0.714462;
35  0.704696;
40  0.688360
];

tilt_vfh = [
0   0.684127;
5   0.701587;
10  0.691402;
20  0.691336;
30  0.632341;
40  0.622288
];

tilt = tilt_vfh;
%tilt = tilt_sap772_nonorm;
%tilt = tilt_sap772_pcanorm;
%tilt = tilt_sap772_rollnorm;

figure(1)
set(gca, 'FontSize', 12)
plot(tilt(:,1), tilt(:,2)*100, 'LineWidth', 1.25)
grid
xlim([0,40])
ylim([0, 100])
xlabel('Tilt Angle of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)


figure(2)
set(gca, 'FontSize', 12)
plot(tilt(:,1), [tilt_sap772_pcanorm(:,2)*100, tilt_sap772_rollnorm(:,2)*100, tilt_sap772_nonorm(:,2)*100], 'LineWidth', 1.25)
grid
xlim([0,40])
ylim([0, 100])
xlabel('Tilt Angle of Unknown Object View')
ylabel('Recall (%)')
l__ = legend('SAP-7-7-2 PCA normalized', 'SAP-7-7-2 roll compensation', 'SAP-7-7-2 not normalized', 'Location', 'SouthWest');
set(l__, 'Box', 'off')
set(l__, 'FontSize', 12)
