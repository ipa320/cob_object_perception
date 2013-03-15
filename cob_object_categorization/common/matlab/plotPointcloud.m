clear all
close all

% I = imread('G:/ObjectDataNew/TrainingData/Books/Book002/color', 'bmp');
% C = load('G:/ObjectDataNew/TrainingData/Books/Book002/coordinates.txt');
I = imread('G:/ObjectDataNew/TrainingData/CoffeePot/pot002/color', 'bmp');
C = load('G:/ObjectDataNew/TrainingData/CoffeePot/pot002/coordinates.txt');

downscale = 6;
Cd = zeros(size(C,1)/downscale, 3);
Id = zeros(size(C,1)/downscale, 3);
stride = size(I,1);
j = 1;
for (i=1:size(C,1))
    if (mod(i, downscale) == 0)
        Cd(j,:) = C(i,:);
        u = mod(i, stride);
        if (u==0)
            u = stride;
        end
        v = (i-u)/stride;
        Id(j,1) = double(I(v+1,u,1))/255.0;
        Id(j,2) = double(I(v+1,u,2))/255.0;
        Id(j,3) = double(I(v+1,u,3))/255.0;
        j = j+1;
    end
end

scatter3(Cd(:,1), Cd(:,3), -Cd(:,2), 20, Id, 'filled')

xlabel('x');
ylabel('z');
zlabel('y');

axis equal