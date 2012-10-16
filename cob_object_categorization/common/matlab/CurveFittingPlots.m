close all

for i=0:5
    %Folder = ['../files/IPAData/Classifier/Statistics/GlobalFP_CurveFitting_book_1_0(', num2str(i), ')_SensorData.txt'];
    Folder = ['../files/GlobalFP_CurveFitting(', num2str(i), ')_SensorData.txt'];
    CurveFitData = load('-ascii', Folder)
    %Folder = ['../files/IPAData/Classifier/Statistics/GlobalFP_CurveFitting_book_1_0(', num2str(i), ')_PolyParams.txt'];
    Folder = ['../files/GlobalFP_CurveFitting(', num2str(i), ')_PolyParams.txt'];
    PolyParams = load('-ascii', Folder)
    
    figure
    plot(CurveFitData(:,1), CurveFitData(:,2), 'r.', 'DisplayName', 'Sensor data')
    hold on
    fplot([num2str(PolyParams(1)), '*x+', num2str(PolyParams(2)), '*x^2+', num2str(PolyParams(3)), '*x^3+', num2str(PolyParams(4)), '*x^4'], [-1, 1], 'b')
%     if (i==0)
%         fplot('-0.641683*x+0.0530485*x^2+0.000154582*x^3', [-60,60], 'b')
%     elseif (i==1)
%         fplot('0.0923168*x+0.0490592*x^2-0.000343111*x^3',[-60,60], 'b')
%     elseif (i==2)
%         fplot('0.230813*x+0.0438667*x^2-0.000387329*x^3',[-60,60], 'b')
%     elseif (i==3)
%         fplot('0.280204*x+0.0497363*x^2-0.000187069*x^3',[-60,60], 'b')
%     end
    %title('4th order curve fitting along PCAs strongest eigenvector direction for object "ball"');
    xlabel('s [mm/mm]   (normalized to s_{max} - s_{min})')
    ylabel('z [mm/mm]   (normalized to s_{max} - s_{min})')
    ylim([-1, 1])
    legend('Sensor data', 'Least Squares curve fitting')
end