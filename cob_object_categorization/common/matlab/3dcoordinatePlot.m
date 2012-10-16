close all

A = load('../files/coordinates.txt');
B = load('../files/coordinatestf.txt');

scatter3(A(:,1), A(:,2), A(:,3), '.')
figure
scatter3(B(:,1), B(:,2), B(:,3), '.')