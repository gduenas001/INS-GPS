clear
close all
clc

load('IJJRR_revised_paper.mat')

figure; grid on;
set(gca, 'FontSize',10)
plot(avg_n_L_M, avg_epoch, 'b-', 'linewidth', 2)
xlabel({'Mininum number of landmarks in the preceding horizon ($n^{F^(M)}$)'},'interpreter', 'latex','fontsize', 10)
ylabel({'$M$'},'interpreter', 'latex','fontsize', 10)
legend({'Average number of time epochs in the preceding horizon'},'interpreter', 'latex','fontsize', 10)
xlim([10,35])

figure; grid on;
set(gca, 'FontSize',10)
plot(avg_n_L_M, avg_f_mag, 'r-', 'linewidth', 2)
xlabel({'Mininum number of landmarks in the preceding horizon ($n^{F^(M)}$)'},'interpreter', 'latex','fontsize', 10)
ylabel({'$\eta_{\textit{mag}}$'},'interpreter', 'latex','fontsize', 10)
legend({'Average magnitude of the worst-case fault'},'interpreter', 'latex','fontsize', 10)
xlim([10,35])

figure; grid on;
set(gca, 'FontSize',10)
plot(avg_n_L_M, computational_time, 'k-', 'linewidth', 2)
xlabel({'Mininum number of landmarks in the preceding horizon ($n^{F^(M)}$)'},'interpreter', 'latex','fontsize', 10)
ylabel({'time (Seconds)'},'interpreter', 'latex','fontsize', 10)
legend({'Average time needed to monitor integrity'},'interpreter', 'latex','fontsize', 10)
xlim([10,35])