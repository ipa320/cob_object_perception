% x = [-1 0 1000];
% figure
% hold on
% grid
% for i=-0.1:0.01:0.1
%     %i = i*10;
%     %x(2) = -2*x(3)*i;
%     %x(1) = x(3)*i^2;
%     x
%     functionString = num2str(x(1));
%     for o=1:(size(x,2)-1)
%         functionString = [functionString, '+', num2str(x(o+1)), '*x^', num2str(o)];
%     end
%     fplot(functionString, [-1.2 1.2], 'r');
% end
% x;


%% draws some plots which show the usefulness of the chosen normalization
clear all


% box coordinates
box.halfSideLengths = [30 18 12];
box.coordinates = [ box.halfSideLengths(1)  box.halfSideLengths(2)  box.halfSideLengths(3);
                    box.halfSideLengths(1)  box.halfSideLengths(2) -box.halfSideLengths(3);
                    box.halfSideLengths(1) -box.halfSideLengths(2)  box.halfSideLengths(3);
                    box.halfSideLengths(1) -box.halfSideLengths(2) -box.halfSideLengths(3);
                   -box.halfSideLengths(1)  box.halfSideLengths(2)  box.halfSideLengths(3);
                   -box.halfSideLengths(1)  box.halfSideLengths(2) -box.halfSideLengths(3);
                   -box.halfSideLengths(1) -box.halfSideLengths(2)  box.halfSideLengths(3);
                   -box.halfSideLengths(1) -box.halfSideLengths(2) -box.halfSideLengths(3)];
box.center = [0 0 0];

% virtual camera setting
box.cameraPosition = [];
box.cameraFrame = {};
box.viewCentroidsTheory = [];
box.viewCentroidsSampled = [];
box.viewEigenvectorsSampled = {};
box.transformedViews = {};
for pan = pi/32:pi/16:(pi/2-pi/32)         %pi/32:pi/16:(pi/2-pi/32)         %pan = 4*pi/36:3*pi/36:14*pi/36
    for tilt = pi/12:pi/6:(pi/2-pi/12)             %4*pi/32:pi/32:(pi/2-pi/16)                      %tilt = pi/6
        cam.pan = pan;
        cam.tilt = tilt;
        
        % theoretical centroids
        box.viewCentroidsTheory = [box.viewCentroidsTheory; [box.halfSideLengths(1) * 4*sin(cam.tilt)*box.halfSideLengths(2)*box.halfSideLengths(3), box.halfSideLengths(2) * 4*cos(cam.tilt)*sin(cam.pan)*box.halfSideLengths(1)*box.halfSideLengths(3), box.halfSideLengths(3) * 4*cos(cam.tilt)*cos(cam.pan)*box.halfSideLengths(1)*box.halfSideLengths(2)]/(4*sin(cam.tilt)*box.halfSideLengths(2)*box.halfSideLengths(3) + 4*cos(cam.tilt)*sin(cam.pan)*box.halfSideLengths(1)*box.halfSideLengths(3) + 4*cos(cam.tilt)*cos(cam.pan)*box.halfSideLengths(1)*box.halfSideLengths(2))];
        
        % camera position (for drawing some camera positions into the image)
        cameraDistance = 80;
        box.cameraPosition = [box.cameraPosition; [cameraDistance*sin(cam.tilt), cameraDistance*cos(cam.tilt)*sin(cam.pan), cameraDistance*cos(cam.tilt)*cos(cam.pan)]];
        box.cameraFrame = {box.cameraFrame{:}, [cos(cam.tilt), 0, sin(cam.tilt); -sin(cam.tilt)*sin(cam.pan), cos(cam.pan), cos(cam.tilt)*sin(cam.pan); -sin(cam.tilt)*cos(cam.pan), -sin(cam.pan), cos(cam.tilt)*cos(cam.pan)]};
         
        % percentage of points in dependence of the camera angles
        box.threshold.xp = sin(cam.tilt);
        box.threshold.yp = (cos(cam.tilt) * sin(cam.pan));
        box.threshold.zp = (cos(cam.tilt) * cos(cam.pan));

        % sample points from each surface
        box.samplePoints.xp = []; %zeros(box.halfSideLengths(2)*2+box.halfSideLengths(3)*2+2, 3);
        box.samplePoints.xm = zeros(box.halfSideLengths(2)*2+box.halfSideLengths(3)*2+2, 3);
        index = 1;
        for y=-box.halfSideLengths(2):box.halfSideLengths(2)
            for z=-box.halfSideLengths(3):box.halfSideLengths(3)
                if (rand(1) < box.threshold.xp)
                    box.samplePoints.xp = [box.samplePoints.xp; [box.halfSideLengths(1), y, z]];
                end
                box.samplePoints.xm(index, :) = [-box.halfSideLengths(1), y, z];
                index = index + 1;
            end
        end
        box.samplePoints.yp = []; %zeros(box.halfSideLengths(1)*2+box.halfSideLengths(3)*2+2, 3);
        box.samplePoints.ym = zeros(box.halfSideLengths(1)*2+box.halfSideLengths(3)*2+2, 3);
        index = 1;
        for x=-box.halfSideLengths(1):box.halfSideLengths(1)
            for z=-box.halfSideLengths(3):box.halfSideLengths(3)
                if (rand(1) < box.threshold.yp)
                    box.samplePoints.yp = [box.samplePoints.yp; [x, box.halfSideLengths(2), z]];
                end
                box.samplePoints.ym(index, :) = [x, -box.halfSideLengths(2), z];
                index = index + 1;
            end
        end
        box.samplePoints.zp = []; %zeros(box.halfSideLengths(1)*2+box.halfSideLengths(2)*2+2, 3);
        box.samplePoints.zm = zeros(box.halfSideLengths(1)*2+box.halfSideLengths(2)*2+2, 3);
        index = 1;
        for x=-box.halfSideLengths(1):box.halfSideLengths(1)
            for y=-box.halfSideLengths(2):box.halfSideLengths(2)
                if (rand(1) < box.threshold.zp)
                    box.samplePoints.zp = [box.samplePoints.zp; [x, y, box.halfSideLengths(3)]];
                end
                box.samplePoints.zm(index, :) = [x, y, -box.halfSideLengths(3)];
                index = index + 1;
            end
        end

        % centroid of the sampled points
        box.viewCentroidsSampled = [box.viewCentroidsSampled; mean([box.samplePoints.xp; box.samplePoints.yp; box.samplePoints.zp], 1)];
        
        % pca -> normalize coordinate axes
        [eigenvectors, temp, eigenvalues] = princomp([box.samplePoints.xp; box.samplePoints.yp; box.samplePoints.zp]);
        box.viewEigenvectorsSampled = {box.viewEigenvectorsSampled{:}, eigenvectors};
        box.transformedViews = {box.transformedViews{:}, temp};
    end
end

% compute the centroids which are theoretically possible
box.viewCentroidTheory = [box.halfSideLengths(1) * 4*box.halfSideLengths(2)*box.halfSideLengths(3), box.halfSideLengths(2) * 4*box.halfSideLengths(1)*box.halfSideLengths(3), box.halfSideLengths(3) * 4*box.halfSideLengths(1)*box.halfSideLengths(2)]/(4*box.halfSideLengths(2)*box.halfSideLengths(3) + 4*box.halfSideLengths(1)*box.halfSideLengths(3) + 4*box.halfSideLengths(1)*box.halfSideLengths(2));



% cam.pan = pi/4;
% cam.tilt = pi/6;
% 
% % percentage of points in dependence of the camera angles
% box.threshold.xp = sin(cam.tilt);
% box.threshold.yp = sqrt(cos(cam.tilt) * sin(cam.pan));
% box.threshold.zp = sqrt(cos(cam.tilt) * cos(cam.pan));
% 
% % sample points from each surface
% box.samplePoints.xp = []; %zeros(box.halfSideLengths(2)*2+box.halfSideLengths(3)*2+2, 3);
% box.samplePoints.xm = zeros(box.halfSideLengths(2)*2+box.halfSideLengths(3)*2+2, 3);
% index = 1;
% for y=-box.halfSideLengths(2):box.halfSideLengths(2)
%     for z=-box.halfSideLengths(3):box.halfSideLengths(3)
%         if (rand(1) < box.threshold.xp)
%             box.samplePoints.xp = [box.samplePoints.xp; [box.halfSideLengths(1), y, z]];
%         end
%         box.samplePoints.xm(index, :) = [-box.halfSideLengths(1), y, z];
%         index = index + 1;
%     end
% end
% box.samplePoints.yp = []; %zeros(box.halfSideLengths(1)*2+box.halfSideLengths(3)*2+2, 3);
% box.samplePoints.ym = zeros(box.halfSideLengths(1)*2+box.halfSideLengths(3)*2+2, 3);
% index = 1;
% for x=-box.halfSideLengths(1):box.halfSideLengths(1)
%     for z=-box.halfSideLengths(3):box.halfSideLengths(3)
%         if (rand(1) < box.threshold.yp)
%             box.samplePoints.yp = [box.samplePoints.yp; [x, box.halfSideLengths(2), z]];
%         end
%         box.samplePoints.ym(index, :) = [x, -box.halfSideLengths(2), z];
%         index = index + 1;
%     end
% end
% box.samplePoints.zp = []; %zeros(box.halfSideLengths(1)*2+box.halfSideLengths(2)*2+2, 3);
% box.samplePoints.zm = zeros(box.halfSideLengths(1)*2+box.halfSideLengths(2)*2+2, 3);
% index = 1;
% for x=-box.halfSideLengths(1):box.halfSideLengths(1)
%     for y=-box.halfSideLengths(2):box.halfSideLengths(2)
%         if (rand(1) < box.threshold.zp)
%             box.samplePoints.zp = [box.samplePoints.zp; [x, y, box.halfSideLengths(3)]];
%         end
%         box.samplePoints.zm(index, :) = [x, y, -box.halfSideLengths(3)];
%         index = index + 1;
%     end
% end


% after transformation
figure
hold on
axis equal
axis off
grid
% xlabel('x')
% ylabel('y')
% zlabel('z')


% surfaces
% surf([box.halfSideLengths(1), -box.halfSideLengths(1)], [box.halfSideLengths(2), -box.halfSideLengths(2)], [box.halfSideLengths(3), box.halfSideLengths(3); box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)
% surf([box.halfSideLengths(1), -box.halfSideLengths(1)], [box.halfSideLengths(2), -box.halfSideLengths(2)], -[box.halfSideLengths(3), box.halfSideLengths(3); box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)
% surf([box.halfSideLengths(1), box.halfSideLengths(1)+0.001], [box.halfSideLengths(2), -box.halfSideLengths(2)], [-box.halfSideLengths(3), box.halfSideLengths(3); -box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)
% surf([-box.halfSideLengths(1), -box.halfSideLengths(1)-0.001], [box.halfSideLengths(2), -box.halfSideLengths(2)], [-box.halfSideLengths(3), box.halfSideLengths(3); -box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)
% surf([box.halfSideLengths(1), -box.halfSideLengths(1)], [box.halfSideLengths(2), box.halfSideLengths(2)+0.001], [-box.halfSideLengths(3), -box.halfSideLengths(3); box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)
% surf([box.halfSideLengths(1), -box.halfSideLengths(1)], [-box.halfSideLengths(2), -box.halfSideLengths(2)-0.001], [-box.halfSideLengths(3), -box.halfSideLengths(3); box.halfSideLengths(3), box.halfSideLengths(3)], [1 1; 1 1]*255, 'FaceAlpha', 0.4)

% corner points
% scatter3(box.coordinates(:,1), box.coordinates(:,2), box.coordinates(:,3))

% center point
scatter3(box.center(:,1), box.center(:,2), box.center(:,3), 'k.')

%scatter3(box.viewCentroidTheory(:,1), box.viewCentroidTheory(:,2), box.viewCentroidTheory(:,3), 'b.')

% scatter3(box.viewCentroidsTheory(:,1), box.viewCentroidsTheory(:,2), box.viewCentroidsTheory(:,3), 'r.')

%scatter3(box.viewCentroidsSampled(:,1), box.viewCentroidsSampled(:,2), box.viewCentroidsSampled(:,3), 'k.')

% coordinate system
set(gca,'ColorOrder',[1 0 0;0 1 0;0 0 1]);
for i=1:size(box.viewEigenvectorsSampled, 2)
    displayScale = 4;
    %plot3([box.center(1)*ones(1,3); box.center(1)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(1,:)],          [box.center(2)*ones(1,3); box.center(2)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(2,:)],          [box.center(3)*ones(1,3); box.center(3)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(3,:)], 'LineWidth', 1.5);
    %plot3([box.viewCentroidsSampled(i,1)*ones(1,3); box.viewCentroidsSampled(i,1)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(1,:)],          [box.viewCentroidsSampled(i,2)*ones(1,3); box.viewCentroidsSampled(i,2)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(2,:)],          [box.viewCentroidsSampled(i,3)*ones(1,3); box.viewCentroidsSampled(i,3)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(3,:)], 'LineWidth', 2);
    %plot3([box.viewCentroidsTheory(i,1)*ones(1,3); box.viewCentroidsTheory(i,1)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(1,:)],          [box.viewCentroidsTheory(i,2)*ones(1,3); box.viewCentroidsTheory(i,2)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(2,:)],          [box.viewCentroidsTheory(i,3)*ones(1,3); box.viewCentroidsTheory(i,3)*ones(1,3)+displayScale*box.viewEigenvectorsSampled{i}(3,:)]);
    displayScale = 4;
    plot3([box.center(1)*ones(1,3); box.center(1)*ones(1,3)+displayScale*box.cameraFrame{i}(1,:)],    [box.center(2)*ones(1,3); box.center(2)*ones(1,3)+displayScale*box.cameraFrame{i}(2,:)],          [box.center(3)*ones(1,3); box.center(3)*ones(1,3)+displayScale*box.cameraFrame{i}(3,:)]);
    %plot3([box.viewCentroidsSampled(i,1)*ones(1,3); box.viewCentroidsSampled(i,1)*ones(1,3)+displayScale*box.cameraFrame{i}(1,:)],          [box.viewCentroidsSampled(i,2)*ones(1,3); box.viewCentroidsSampled(i,2)*ones(1,3)+displayScale*box.cameraFrame{i}(2,:)],          [box.viewCentroidsSampled(i,3)*ones(1,3); box.viewCentroidsSampled(i,3)*ones(1,3)+displayScale*box.cameraFrame{i}(3,:)]);
    %plot3([box.viewCentroidsTheory(i,1)*ones(1,3); box.viewCentroidsTheory(i,1)*ones(1,3)+displayScale*box.cameraFrame{i}(1,:)],          [box.viewCentroidsTheory(i,2)*ones(1,3); box.viewCentroidsTheory(i,2)*ones(1,3)+displayScale*box.cameraFrame{i}(2,:)],          [box.viewCentroidsTheory(i,3)*ones(1,3); box.viewCentroidsTheory(i,3)*ones(1,3)+displayScale*box.cameraFrame{i}(3,:)], '-.');
end

% camera viewpoints
% scatter3(box.cameraPosition(:,1), box.cameraPosition(:,2), box.cameraPosition(:,3), 'k.')
% for i=1:size(box.cameraPosition,1)
%     plot3([box.cameraPosition(i,1); box.cameraPosition(i,1)*0.8], [box.cameraPosition(i,2); box.cameraPosition(i,2)*0.8], [box.cameraPosition(i,3); box.cameraPosition(i,3)*0.8], 'k')
% end

% i=14;
% s = i/size(box.transformedViews, 2);
% scatter3(box.transformedViews{i}(:,1), box.transformedViews{i}(:,2), box.transformedViews{i}(:,3), 20, [s, 1-s, 1-s^2], 'filled');

% sample points
%scatter3(box.samplePoints.xp(:,1), box.samplePoints.xp(:,2), box.samplePoints.xp(:,3), 'b.')
%scatter3(box.samplePoints.xm(:,1), box.samplePoints.xm(:,2), box.samplePoints.xm(:,3), 'b.')
%scatter3(box.samplePoints.yp(:,1), box.samplePoints.yp(:,2), box.samplePoints.yp(:,3), 'b.')
%scatter3(box.samplePoints.ym(:,1), box.samplePoints.ym(:,2), box.samplePoints.ym(:,3), 'b.')
%scatter3(box.samplePoints.zp(:,1), box.samplePoints.zp(:,2), box.samplePoints.zp(:,3), 'b.')
%scatter3(box.samplePoints.zm(:,1), box.samplePoints.zm(:,2), box.samplePoints.zm(:,3), 'b.')

figure
hold on
axis equal
grid
xlabel('x')
ylabel('y')
zlabel('z')

for i=1:size(box.transformedViews, 2)
    s = i/size(box.transformedViews, 2);
    scatter3(box.transformedViews{i}(:,1), box.transformedViews{i}(:,2), box.transformedViews{i}(:,3), 20, [s, 1-s, 1-s^2], 'filled');
end