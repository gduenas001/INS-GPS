clear
%close all
clc

load('IJJRR_revised_paper.mat')

set_of_min_n_L_M=[5 10 12 20 25 30];

figure; grid on;
set(gca, 'FontSize',10)
plot(set_of_min_n_L_M, avg_n_L_M, 'r-', 'linewidth', 2)
xlabel({'Min. assoc. in the horizon, $n_{\textit{min}}^{\textit{A}^{\textit{(M)}}}$'},'interpreter', 'latex','fontsize', 10)
ylabel({'$n_k^{\textit{A}^{\textit{(M)}}}$'},'interpreter', 'latex','fontsize', 10)
legend({'Avg. assoc. in the horizon'},'interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
grid on;
xlim([5,30])

figure; grid on;
set(gca, 'FontSize',10)
plot(set_of_min_n_L_M, avg_epoch, 'b-', 'linewidth', 2)
xlabel({'Min. assoc. in the horizon, $n_{\textit{min}}^{\textit{A}^{\textit{(M)}}}$'},'interpreter', 'latex','fontsize', 10)
ylabel({'$M$'},'interpreter', 'latex','fontsize', 10)
legend({'Avg. epochs in the horizon'},'interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
grid on;
xlim([5,30])

figure; grid on;
set(gca, 'FontSize',10)
plot(set_of_min_n_L_M, avg_f_mag, 'k-', 'linewidth', 2)
xlabel({'Min. assoc. in the horizon, $n_{\textit{min}}^{\textit{A}^{\textit{(M)}}}$'},'interpreter', 'latex','fontsize', 10)
ylabel({'$\eta_{\textit{mag}}$'},'interpreter', 'latex','fontsize', 10)
legend({'Avg. mag. of worst-case fault'},'interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
grid on;
xlim([5,30])

figure; grid on;
set(gca, 'FontSize',10)
plot(set_of_min_n_L_M, computational_time, 'k-', 'linewidth', 2)
xlabel({'Min. assoc. in the horizon, $n_{\textit{min}}^{\textit{A}^{\textit{(M)}}}$'},'interpreter', 'latex','fontsize', 10)
ylabel({'time (Seconds)'},'interpreter', 'latex','fontsize', 10)
legend({'Avg. time to monitor integrity'},'interpreter', 'latex','fontsize', 10)
set(gca,'TickLabelInterpreter','latex','fontsize', 10)
grid on;
xlim([5,30])