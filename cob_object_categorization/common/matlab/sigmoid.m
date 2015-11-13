f = [];
mx = -1:0.01:1;
alphas = 0.2:0.2:5;
for (alpha = alphas)
    f = [f; exp(alpha*mx)./(exp(alpha*mx)+exp(-alpha*mx))];
end

figure
axes
set(gca, 'FontSize', 12)
hold on
alphanum = size(alphas,2);
colormapping = colormap(jet);
a = 63/(alphas(end)-alphas(1));
b = 1-a*alphas(1);
for (i=1:alphanum);
    % y = ax + b
    % 
    % 1 = a*alphas(1) + b
    % b = 1-a*alphas(1)
    % 
    % 64 = a*(alphas(end)-alphas(1)) + 1
    % 63/(alphas(end)-alphas(1)) = a

    p = round(a*alphas(i)+b);
    plot(mx, f(i,:), 'color', colormapping(p,:))
end
grid
caxis([alphas(1), alphas(end)])
colorbar
xlabel('m_k(x)')
ylabel('L(a_k|x)')


