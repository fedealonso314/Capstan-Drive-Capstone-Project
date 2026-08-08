%% weibull_mle_censored.m
% Two-parameter Weibull MLE with right-censored (suspended) observations.
% Capstan drive endurance testing - rope fatigue life.
%
% Data: Run 1 failed @ 14,000 | Run 2 susp. @ 20,000
%       Run 3 susp. @ 20,000  | Run 4 failed @ 18,000
%
% No toolboxes required (fzero, gamma and log are base MATLAB).

clear; clc; format long g

%% ---------------- 1. Input data ----------------
t      = [14000; 18000; 20000; 20000];   % time on test (cycles)
failed = [true;  true;  false; false];   % true = failure, false = suspension

n   = numel(t);
nf  = sum(failed);
lnt = log(t);
A   = mean(lnt(failed));                 % (1/nf) * sum_{i in F} ln t_i

fprintf('n = %d, n_f = %d, mean(ln t_failed) = %.6f\n\n', n, nf, A);

%% ---------------- 2. Solve Eq. 7 for beta ----------------
% g(beta) = sum(t^b .* ln t)/sum(t^b) - 1/b - A = 0
g = @(b) sum(t.^b .* lnt) / sum(t.^b) - 1/b - A;

% Show that g is monotonic and brackets the root
fprintf('beta      g(beta)\n');
for b = [0.5 1 2 3 4 5 6 8 12]
    fprintf('%5.2f  %+12.6f\n', b, g(b));
end

beta_hat = fzero(g, [0.1 50]);

%% ---------------- 3. Recover eta (Eq. 8) ----------------
eta_hat = ( sum(t.^beta_hat) / nf )^(1/beta_hat);

%% ---------------- 4. Derived life metrics ----------------
MTTF = eta_hat * gamma(1 + 1/beta_hat);              % Eq. 9
B10  = eta_hat * (-log(0.9))^(1/beta_hat);
R    = @(x) exp(-(x./eta_hat).^beta_hat);

%% ---------------- 5. Confidence intervals ----------------
% Negative log-likelihood parameterised in log-space: p = [ln beta, ln eta]
negLL = @(p) -( sum( log(exp(p(1))) - exp(p(1))*p(2) ...
                     + (exp(p(1))-1)*lnt(failed) ) ...
                - sum( (t./exp(p(2))).^exp(p(1)) ) );

p0 = [log(beta_hat); log(eta_hat)];
h  = 1e-5;
H  = zeros(2);                                    % observed Fisher information
for i = 1:2
    for j = 1:2
        ei = zeros(2,1); ei(i) = h;
        ej = zeros(2,1); ej(j) = h;
        H(i,j) = ( negLL(p0+ei+ej) - negLL(p0+ei-ej) ...
                 - negLL(p0-ei+ej) + negLL(p0-ei-ej) ) / (4*h*h);
    end
end
C  = inv(H);                                      % covariance in log-space
se = sqrt(diag(C));

C_level = 0.90;
z = -sqrt(2)*erfcinv(2*(1 - (1-C_level)/2));      % = norminv(0.95) = 1.6449

beta_CI = beta_hat * exp([-1 1] * z * se(1));
eta_CI  = eta_hat  * exp([-1 1] * z * se(2));

%% ---------------- 6. Report ----------------
fprintf('\n--- Results (C = %.0f%%) ---\n', 100*C_level);
fprintf('beta_hat = %8.4f   (90%% CI: %6.2f -- %6.2f)\n', beta_hat, beta_CI);
fprintf('eta_hat  = %8.0f   (90%% CI: %6.0f -- %6.0f) cycles\n', eta_hat, eta_CI);
fprintf('MTTF     = %8.0f cycles\n', MTTF);
fprintf('B10      = %8.0f cycles\n', B10);
fprintf('R(14000) = %6.2f%%   R(20000) = %6.2f%%\n', 100*R(14000), 100*R(20000));
fprintf('R_LCB(14000) using eta_LCB = %6.2f%%\n', ...
        100*exp(-(14000/eta_CI(1))^beta_hat));

fprintf('\nObserved Fisher information (log-space):\n'); disp(H);
fprintf('Covariance matrix:\n'); disp(C);
fprintf('SE(ln beta) = %.4f   SE(ln eta) = %.4f\n', se(1), se(2));

%% ---------------- 7. Weibull probability plot ----------------
% Median ranks adjusted for suspensions (Johnson rank-adjustment).
% With suspensions at the END of the ordered list, adjusted ranks equal
% simple ranks, so this reduces to Bernard's approximation on the failures.
tf   = sort(t(failed));
i    = (1:numel(tf))';
MR   = (i - 0.3) / (n + 0.4);
xdat = log(tf);
ydat = log(-log(1 - MR));

tline = linspace(0.6*min(t), 1.4*max(t), 200)';
yline = beta_hat * (log(tline) - log(eta_hat));

figure;
plot(xdat, ydat, 'ko', 'MarkerFaceColor','k', 'MarkerSize', 7); hold on;
plot(log(tline), yline, 'b-', 'LineWidth', 1.4);
xlabel('ln(cycles)');
ylabel('ln(-ln(1-F))');
title(sprintf('Weibull probability plot: \\beta = %.2f, \\eta = %.0f cycles', ...
      beta_hat, eta_hat));
legend('Failures (median rank)', 'MLE fit', 'Location','southeast');
grid on; box on;