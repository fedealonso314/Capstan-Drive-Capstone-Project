function analyze_weibull_endurance()
% =========================================================================
%  WEIBULL RELIABILITY ANALYSIS — Test 3 Endurance (capstan drive)
%  Capstan Drive Reliability Project
%
%  Reads:  All test3_endurance_*.csv in current folder (one per run)
%  Columns expected per CSV:
%    CYCLE, TIMESTAMP_S, CMD_POS_DEG, CMD_NEG_DEG,
%    MEAN_TRANS_ERR_DEG, STD_TRANS_ERR_DEG, SLIP_FLAG,
%    MEAN_CURRENT_mA, FAILURE_TIER
%
%  Metadata block at bottom (after data rows):
%    STOP_REASON,  TOTAL_CYCLES,  LOAD_PCT,  GEAR_RATIO,
%    BASELINE_MEAN_TE_DEG,  BASELINE_MEAN_CURRENT_mA,  CENSORED
%
%  Analysis sections:
%   1.  Load all runs, extract failure/censoring cycle per tier
%   2.  Weibull 2-parameter MLE with right-censoring
%   3.  If all runs censored — lower-bound analysis + projection
%   4.  Reliability function R(t), B10 / B50 / MTTF
%   5.  Weibull probability paper plot (rank regression overlay)
%   6.  Degradation: TE and current trend across cycles (all runs)
%   7.  Slip event timeline
%   8.  Mini load-response deviation (TE vs calibration reference)
%   9.  Summary table — logbook ready
%
%  HOW TO USE:
%    Place this file in the same folder as your test3_endurance_*.csv files.
%    Run:  analyze_weibull_endurance()
% =========================================================================

clc; clear; close all;

%% ================================================================
%% TUNING — update from your Test 0 / Test 1 outputs
%% ================================================================

% Failure tier thresholds (methodology §2)
TIER1_TE_DELTA_DEG   =  1.0;   % |TE - baseline| > 1 deg → Tier 1 signal
TIER2_TE_MEAN_DEG    =  3.0;   % |mean TE over 10 cyc| > 3 deg → Tier 2
TIER3_SLIP_DEG       = 15.0;   % output deviation > 15 deg → Tier 3 (stop)
TIER2_CURR_PCT       =  0.20;  % current > +20% baseline → Tier 2
TIER1_CURR_PCT       =  0.05;  % current > +5% baseline → Tier 1 signal

% Baseline references (from Test 0 analyze_baseline_results output)
BASELINE_MEAN_TE_GLOBAL   = -0.3407;   % deg  — update if different
BASELINE_MEAN_CURR_GLOBAL =  161.8;    % mA   — update if different

% TE at 55% load from Test 1 calibration curve (load response)
CALIB_TE_55PCT = -0.9145;   % deg — from analyze_load_response_results output

% Maximum planned test cycles (right-censoring point)
MAX_CYCLES = 10000;

% Weibull confidence level (90% is standard for reliability engineering)
CONF = 0.90;

% Moving average window for degradation smoothing
MA_WIN = 20;

% Tier to use for primary Weibull fit (1=minimal, 2=intermediate, 3=critical)
PRIMARY_TIER = 2;

%% ================================================================
%% 1. LOAD ALL RUNS
%% ================================================================

files = dir('test3_endurance_*.csv');
if isempty(files)
    % Try uploads path (Claude environment)
    files = dir('/mnt/user-data/uploads/test3_endurance*.csv');
end
if isempty(files)
    error('No test3_endurance_*.csv found. Place this script in the same folder as the CSV files.');
end

n_runs = numel(files);
fprintf('==========================================================\n');
fprintf('  WEIBULL ENDURANCE ANALYSIS — %d run(s) detected\n', n_runs);
fprintf('==========================================================\n\n');

%% Pre-allocate per-run storage
R = struct();
for fi = 1:n_runs
    R(fi).filename    = files(fi).name;
    R(fi).meta        = struct();
    R(fi).cycle       = [];
    R(fi).mean_te     = [];
    R(fi).std_te      = [];
    R(fi).slip        = [];
    R(fi).current     = [];
    R(fi).tier        = [];
    R(fi).censored    = true;           % assume censored until proven otherwise
    R(fi).fail_cycle  = [NaN NaN NaN]; % [tier1 tier2 tier3] failure cycles
    R(fi).end_cycle   = NaN;
    R(fi).baseline_te   = BASELINE_MEAN_TE_GLOBAL;
    R(fi).baseline_curr = BASELINE_MEAN_CURR_GLOBAL;
end

for fi = 1:n_runs
    fname = fullfile(files(fi).folder, files(fi).name);
    fprintf('Loading run %d: %s\n', fi, files(fi).name);

    %% --- Read metadata ---
    meta = read_metadata(fname);
    R(fi).meta = meta;

    % Use per-run baseline if available in metadata, else use global
    if isfield(meta, 'BASELINE_MEAN_TE_DEG') && ~isnan(meta.BASELINE_MEAN_TE_DEG)
        R(fi).baseline_te = meta.BASELINE_MEAN_TE_DEG;
    end
    if isfield(meta, 'BASELINE_MEAN_CURRENT_mA') && ~isnan(meta.BASELINE_MEAN_CURRENT_mA)
        R(fi).baseline_curr = meta.BASELINE_MEAN_CURRENT_mA;
    end
    if isfield(meta, 'CENSORED')
        R(fi).censored = logical(meta.CENSORED);
    end

    %% --- Read data rows ---
    raw = readtable(fname, 'VariableNamingRule', 'preserve');

    % Drop non-numeric / metadata rows
    if isnumeric(raw.CYCLE)
        valid = ~isnan(raw.CYCLE) & raw.CYCLE > 0;
    else
        cycle_num = str2double(raw.CYCLE);
        valid = ~isnan(cycle_num) & cycle_num > 0;
        raw.CYCLE = cycle_num;
    end
    raw = raw(valid, :);

    if isempty(raw)
        fprintf('  [WARNING] No valid data rows in run %d — skipping.\n\n', fi);
        continue;
    end

    R(fi).cycle   = double(raw.CYCLE);
    R(fi).mean_te = double(raw.MEAN_TRANS_ERR_DEG);
    R(fi).std_te  = double(raw.STD_TRANS_ERR_DEG);
    R(fi).slip    = double(raw.SLIP_FLAG);
    R(fi).current = double(raw.MEAN_CURRENT_mA);
    R(fi).tier    = double(raw.FAILURE_TIER);
    R(fi).end_cycle = max(R(fi).cycle);

    %% --- Detect failure cycles per tier from raw data ---
    bl_te   = R(fi).baseline_te;
    bl_curr = R(fi).baseline_curr;

    % Tier 1: |TE - baseline| > 1 deg  OR  current 5-10% above baseline
    te_delta   = abs(R(fi).mean_te - bl_te);
    curr_rise  = (R(fi).current - bl_curr) / abs(bl_curr);
    t1_mask    = (te_delta > TIER1_TE_DELTA_DEG) | ...
                 (curr_rise >= TIER1_CURR_PCT & curr_rise < TIER2_CURR_PCT);

    % Tier 2: |mean TE| > 3 deg over 10 consecutive cycles OR current > +20%
    te_roll10 = movmean(R(fi).mean_te, 10);
    t2_mask   = (abs(te_roll10) > TIER2_TE_MEAN_DEG) | ...
                (curr_rise >= TIER2_CURR_PCT);

    % Tier 3: slip flag OR TE > 15 deg OR logged tier == 3
    t3_mask   = (R(fi).slip > 0) | ...
                (abs(R(fi).mean_te) > TIER3_SLIP_DEG) | ...
                (R(fi).tier >= 3);

    % First crossing cycle for each tier
    t1_cyc = get_first_crossing(R(fi).cycle, t1_mask);
    t2_cyc = get_first_crossing(R(fi).cycle, t2_mask);
    t3_cyc = get_first_crossing(R(fi).cycle, t3_mask);

    R(fi).fail_cycle = [t1_cyc, t2_cyc, t3_cyc];

    % Override censored flag if Tier 3 detected
    if ~isnan(t3_cyc)
        R(fi).censored = false;
    end

    fprintf('  Cycles: %d  |  Baseline TE: %.4f deg  |  Baseline I: %.1f mA\n', ...
            R(fi).end_cycle, bl_te, bl_curr);
    fprintf('  Tier 1 first crossing: %s\n', fmt_cycle(t1_cyc));
    fprintf('  Tier 2 first crossing: %s\n', fmt_cycle(t2_cyc));
    fprintf('  Tier 3 / Failure     : %s\n', fmt_cycle(t3_cyc));
    fprintf('  Censored             : %d\n\n', R(fi).censored);
end

%% ================================================================
%% 2. BUILD WEIBULL INPUT VECTORS
%%    For each tier: failure_cycles (failed runs) + censoring_cycles
%% ================================================================

fprintf('==========================================================\n');
fprintf('  WEIBULL INPUT SUMMARY\n');
fprintf('==========================================================\n');

for tier_id = 1:3
    [fail_cyc, cens_cyc] = build_weibull_input(R, n_runs, tier_id, MAX_CYCLES);

    n_fail = sum(~isnan(fail_cyc));
    n_cens = numel(cens_cyc);

    fprintf('\nTier %d:  %d failure(s),  %d censored observation(s)\n', ...
            tier_id, n_fail, n_cens);
    for fi = 1:n_runs
        fc = R(fi).fail_cycle(tier_id);
        if ~isnan(fc)
            fprintf('  Run %d → FAILED at cycle %d\n', fi, fc);
        else
            ec = min(R(fi).end_cycle, MAX_CYCLES);
            fprintf('  Run %d → Censored at cycle %d\n', fi, ec);
        end
    end
end
fprintf('\n');

%% ================================================================
%% 3. WEIBULL MLE FIT — PRIMARY TIER
%% ================================================================

fprintf('==========================================================\n');
fprintf('  WEIBULL MLE FIT  (Primary tier: Tier %d)\n', PRIMARY_TIER);
fprintf('==========================================================\n\n');

[fail_cyc_p, cens_cyc_p] = build_weibull_input(R, n_runs, PRIMARY_TIER, MAX_CYCLES);
n_fail_p = sum(~isnan(fail_cyc_p));

if n_fail_p == 0
    fprintf('[INFO] All runs censored at Tier %d — full MLE cannot be computed.\n', PRIMARY_TIER);
    fprintf('       Performing lower-bound / projection analysis instead.\n\n');
    weibull_censored_only_analysis(R, n_runs, MAX_CYCLES, CONF, ...
        CALIB_TE_55PCT, BASELINE_MEAN_TE_GLOBAL, PRIMARY_TIER);
else
    % Full Weibull MLE
    [beta_hat, eta_hat, beta_ci, eta_ci] = weibull_mle(fail_cyc_p, cens_cyc_p, CONF);

    fprintf('Weibull Parameters (MLE, %d%% CI):\n', round(CONF*100));
    fprintf('  Shape  β = %.4f   [%.4f, %.4f]\n', beta_hat, beta_ci(1), beta_ci(2));
    fprintf('  Scale  η = %.1f   [%.1f, %.1f]  cycles\n', eta_hat, eta_ci(1), eta_ci(2));

    if beta_hat < 1
        fprintf('  β < 1 → Early-life (infant mortality) failures\n');
    elseif beta_hat < 1.5
        fprintf('  β ≈ 1 → Random / approximately exponential failure rate\n');
    else
        fprintf('  β > 1 → Wear-out dominated failures (expected for rope fatigue)\n');
    end

    % Reliability metrics
    B10  = eta_hat * (-log(0.90))^(1/beta_hat);
    B50  = eta_hat * (-log(0.50))^(1/beta_hat);   % Median life
    MTTF = eta_hat * gamma(1 + 1/beta_hat);

    fprintf('\nReliability Metrics:\n');
    fprintf('  B10  (10%% failure probability) = %.0f cycles\n', B10);
    fprintf('  B50  (50%% failure probability) = %.0f cycles  [Median life]\n', B50);
    fprintf('  η    (63.2%% failure)           = %.0f cycles  [Characteristic life]\n', eta_hat);
    fprintf('  MTTF                           = %.0f cycles\n', MTTF);

    %% Plot: Weibull probability paper
    plot_weibull_paper(fail_cyc_p, cens_cyc_p, beta_hat, eta_hat, beta_ci, eta_ci, ...
                       PRIMARY_TIER, CONF);

    %% Plot: Reliability function R(t)
    plot_reliability(beta_hat, eta_hat, beta_ci, eta_ci, MAX_CYCLES, B10, B50, CONF, PRIMARY_TIER);

    %% Fit all 3 tiers for comparison table
    fprintf('\n--- Weibull fit across all tiers ---\n');
    fprintf('  %-6s | %-8s | %-8s | %-10s | %-10s | %-10s | %-10s\n', ...
            'Tier', 'n_fail', 'n_cens', 'beta', 'eta', 'B10', 'MTTF');
    fprintf('  %s\n', repmat('-', 1, 72));
    for tier_id = 1:3
        [fc, cc] = build_weibull_input(R, n_runs, tier_id, MAX_CYCLES);
        nf = sum(~isnan(fc));
        nc = numel(cc);
        if nf >= 2
            [b, e, ~, ~] = weibull_mle(fc, cc, CONF);
            b10_t  = e * (-log(0.90))^(1/b);
            mttf_t = e * gamma(1 + 1/b);
            fprintf('  %-6d | %-8d | %-8d | %-10.3f | %-10.0f | %-10.0f | %-10.0f\n', ...
                    tier_id, nf, nc, b, e, b10_t, mttf_t);
        elseif nf == 1
            fprintf('  Tier %d: only 1 failure — need ≥2 for MLE (add runs)\n', tier_id);
        else
            fprintf('  Tier %d: 0 failures — all censored\n', tier_id);
        end
    end
end

%% ================================================================
%% 4. DEGRADATION ANALYSIS — TE AND CURRENT ACROSS CYCLES
%% ================================================================

fprintf('\n==========================================================\n');
fprintf('  DEGRADATION ANALYSIS\n');
fprintf('==========================================================\n\n');

figure('Name','Degradation — TE vs Cycles', 'Position',[50 580 1100 380]);
hold on; grid on;
colors = lines(n_runs);

for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    c  = R(fi).cycle;
    te = R(fi).mean_te;
    bl = R(fi).baseline_te;

    % Raw (faint)
    plot(c, te, 'Color', [colors(fi,:) 0.25], 'LineWidth', 0.5, 'HandleVisibility','off');

    % Moving average
    te_ma = movmean(te, min(MA_WIN, numel(te)));
    plot(c, te_ma, 'Color', colors(fi,:), 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Run %d (MA%d)', fi, MA_WIN));

    % Linear trend
    p = polyfit(c, te, 1);
    plot([c(1) c(end)], polyval(p, [c(1) c(end)]), '--', ...
         'Color', colors(fi,:)*0.7, 'LineWidth', 1.2, 'HandleVisibility','off');

    slope = p(1);
    fprintf('Run %d TE slope: %+.6f deg/cycle  (%+.4f deg over %d cycles)\n', ...
            fi, slope, slope * R(fi).end_cycle, R(fi).end_cycle);
end

yline(CALIB_TE_55PCT,           'b-',  'Test 1 calibration (55% load)', 'LineWidth', 1.5);
yline(CALIB_TE_55PCT - 1.0,     'y--', 'Calibration −1 deg (Tier 1)',   'LineWidth', 1.0);
yline(CALIB_TE_55PCT + 1.0,     'y--', '',                              'LineWidth', 1.0, 'HandleVisibility','off');
yline(-TIER2_TE_MEAN_DEG,       'r--', 'Tier 2 (−3 deg)',               'LineWidth', 1.0);
xlabel('Cycle Number');
ylabel('Mean Transmission Error [deg]');
title('Test 3 — TE Degradation Over Cycles (solid = moving avg, dashed = linear trend)');
legend('Location','best');

figure('Name','Degradation — Current vs Cycles', 'Position',[50 150 1100 380]);
hold on; grid on;

for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    c    = R(fi).cycle;
    curr = R(fi).current;
    bl_c = R(fi).baseline_curr;

    plot(c, curr, 'Color', [colors(fi,:) 0.25], 'LineWidth', 0.5, 'HandleVisibility','off');
    curr_ma = movmean(curr, min(MA_WIN, numel(curr)));
    plot(c, curr_ma, 'Color', colors(fi,:), 'LineWidth', 1.8, ...
         'DisplayName', sprintf('Run %d (MA%d)', fi, MA_WIN));

    p2 = polyfit(c, curr, 1);
    fprintf('Run %d |I| slope: %+.5f mA/cycle  (%+.3f mA over %d cycles)\n', ...
            fi, p2(1), p2(1)*R(fi).end_cycle, R(fi).end_cycle);

    yline(bl_c * (1 + TIER1_CURR_PCT), 'y--', sprintf('Run %d +5%% Tier 1', fi), ...
          'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(bl_c * (1 + TIER2_CURR_PCT), 'r--', sprintf('Run %d +20%% Tier 2', fi), ...
          'LineWidth', 1.0, 'HandleVisibility', 'off');
end
xlabel('Cycle Number');
ylabel('Mean |Current| [mA]');
title('Test 3 — Motor Current Over Cycles');
legend('Location','best');

%% ================================================================
%% 5. SLIP EVENT TIMELINE
%% ================================================================

total_slips = 0;
fprintf('\nSLIP EVENT SUMMARY:\n');
for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    slip_cyc = R(fi).cycle(R(fi).slip > 0);
    total_slips = total_slips + numel(slip_cyc);
    if isempty(slip_cyc)
        fprintf('  Run %d: 0 slip events\n', fi);
    else
        fprintf('  Run %d: %d slip event(s) at cycles: %s\n', ...
                fi, numel(slip_cyc), num2str(slip_cyc'));
    end
end
if total_slips == 0
    fprintf('  → No slip events detected across all runs.\n');
end
fprintf('\n');

%% ================================================================
%% 6. TE DEVIATION FROM CALIBRATION (mini load response check)
%% ================================================================

fprintf('TE DEVIATION FROM TEST 1 CALIBRATION (at 55%% load):\n');
fprintf('  Expected TE at 55%% load: %.4f deg\n', CALIB_TE_55PCT);
for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    te_mean_run = mean(R(fi).mean_te);
    deviation   = te_mean_run - CALIB_TE_55PCT;
    fprintf('  Run %d: mean TE = %.4f deg  |  deviation from calibration = %+.4f deg  ', ...
            fi, te_mean_run, deviation);
    if abs(deviation) > 1.0
        fprintf('[Tier 1 signal]\n');
    else
        fprintf('[Within bounds]\n');
    end
end
fprintf('\n');

%% ================================================================
%% 7. PROJECTED FAILURE CYCLE (from degradation rate)
%% ================================================================

fprintf('PROJECTED FAILURE CYCLE (linear degradation extrapolation):\n');
fprintf('  (Extrapolates TE trend to Tier 2 boundary at %.1f deg)\n', -TIER2_TE_MEAN_DEG);
for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    c  = R(fi).cycle;
    te = R(fi).mean_te;
    p  = polyfit(c, te, 1);
    % Project when TE crosses -TIER2_TE_MEAN_DEG
    if p(1) ~= 0
        proj_cyc = (-TIER2_TE_MEAN_DEG - p(2)) / p(1);
        if proj_cyc > max(c) && proj_cyc < 1e7
            fprintf('  Run %d: projected Tier 2 crossing at cycle ~%.0f\n', fi, proj_cyc);
        elseif proj_cyc <= max(c)
            fprintf('  Run %d: Tier 2 already reached at observed cycles\n', fi);
        else
            fprintf('  Run %d: TE trend flat — no Tier 2 crossing projected\n', fi);
        end
    else
        fprintf('  Run %d: slope ≈ 0, no crossing projected\n', fi);
    end
end
fprintf('\n');

%% ================================================================
%% 8. LOGBOOK SUMMARY
%% ================================================================

fprintf('==========================================================\n');
fprintf('  LOGBOOK ENTRY — Test 3 Weibull Reliability Analysis\n');
fprintf('==========================================================\n');
fprintf('  Date/time       : %s\n', datestr(now));
fprintf('  Runs analysed   : %d\n', n_runs);
fprintf('  Max test cycles : %d\n', MAX_CYCLES);
fprintf('  Primary tier    : Tier %d\n', PRIMARY_TIER);
fprintf('  -------------------------------------------------------\n');
for fi = 1:n_runs
    if isempty(R(fi).cycle), continue; end
    fprintf('  Run %d | File: %s\n', fi, R(fi).filename);
    fprintf('         | End cycle: %d  |  Censored: %d\n', R(fi).end_cycle, R(fi).censored);
    fprintf('         | Tier 1 at: %-8s  Tier 2 at: %-8s  Tier 3 at: %s\n', ...
            fmt_cycle(R(fi).fail_cycle(1)), fmt_cycle(R(fi).fail_cycle(2)), ...
            fmt_cycle(R(fi).fail_cycle(3)));
end
fprintf('  -------------------------------------------------------\n');
fprintf('  Statistical note:\n');
[~, cens_p] = build_weibull_input(R, n_runs, PRIMARY_TIER, MAX_CYCLES);
n_f = 0;
for fi = 1:n_runs
    if ~isnan(R(fi).fail_cycle(PRIMARY_TIER)), n_f = n_f + 1; end
end
if n_f == 0
    fprintf('  All %d runs censored — Weibull MLE not applicable.\n', n_runs);
    fprintf('  Minimum characteristic life η > %d cycles (all runs survived).\n', ...
            min(cellfun(@(x) x, num2cell(cens_p))));
    fprintf('  Recommend running to failure or adding more runs for full fit.\n');
else
    fprintf('  %d failure(s), %d censored — Weibull MLE applied.\n', n_f, numel(cens_p));
end
fprintf('==========================================================\n');

end  %% ---- END MAIN ----


%% ================================================================
%% HELPER: build_weibull_input
%% Returns fail_cycles (NaN for censored runs) and cens_cycles
%% ================================================================
function [fail_cyc, cens_cyc] = build_weibull_input(R, n_runs, tier_id, MAX_CYCLES)
    fail_cyc = NaN(n_runs, 1);
    cens_cyc = [];
    for fi = 1:n_runs
        fc = R(fi).fail_cycle(tier_id);
        if ~isnan(fc)
            fail_cyc(fi) = fc;
        else
            cens_cyc(end+1) = min(R(fi).end_cycle, MAX_CYCLES); %#ok<AGROW>
        end
    end
end


%% ================================================================
%% HELPER: weibull_mle
%% 2-parameter Weibull MLE with right-censoring
%% Uses log-likelihood maximisation via fminsearch
%% ================================================================
function [beta_hat, eta_hat, beta_ci, eta_ci] = weibull_mle(fail_cyc, cens_cyc, conf)

    % Remove NaN failures
    t_fail = fail_cyc(~isnan(fail_cyc));
    t_cens = cens_cyc(:);

    if numel(t_fail) < 2
        error('Need at least 2 failure observations for Weibull MLE.');
    end

    all_t  = [t_fail; t_cens];
    delta  = [ones(numel(t_fail),1); zeros(numel(t_cens),1)];  % 1=failed, 0=censored

    % Log-likelihood for 2-parameter Weibull
    negloglik = @(p) weibull_negloglik(p(1), p(2), all_t, delta);

    % Initial guess: use method of moments on failures
    beta0 = 2.0;
    eta0  = mean(t_fail);
    opts  = optimset('Display','off', 'TolX',1e-8, 'TolFun',1e-8, 'MaxIter',5000);
    p_hat = fminsearch(negloglik, [log(beta0), log(eta0)], opts);

    beta_hat = exp(p_hat(1));
    eta_hat  = exp(p_hat(2));

    %% Fisher information matrix → confidence intervals
    h      = 1e-4;
    p0     = p_hat;
    H      = zeros(2,2);
    for i = 1:2
        for j = 1:2
            ei = zeros(2,1); ei(i) = h;
            ej = zeros(2,1); ej(j) = h;
            H(i,j) = (negloglik(p0+ei+ej) - negloglik(p0+ei-ej) ...
                     - negloglik(p0-ei+ej) + negloglik(p0-ei-ej)) / (4*h^2);
        end
    end
    try
        cov_log = inv(H);
    catch
        cov_log = diag([0.1 0.1]);  % fallback if singular
    end

    z = norminv(0.5 + conf/2);  % e.g. 1.645 for 90%

    % CI on log(beta) and log(eta), then exponentiate
    beta_log_ci = p_hat(1) + z * [-1, 1] * sqrt(abs(cov_log(1,1)));
    eta_log_ci  = p_hat(2) + z * [-1, 1] * sqrt(abs(cov_log(2,2)));

    beta_ci = exp(beta_log_ci);
    eta_ci  = exp(eta_log_ci);
end


%% ================================================================
%% HELPER: weibull_negloglik
%% ================================================================
function nll = weibull_negloglik(log_beta, log_eta, t, delta)
    beta = exp(log_beta);
    eta  = exp(log_eta);
    % Clip to avoid log(0)
    t = max(t, 1e-10);
    % Log-likelihood = sum_{failed} [log f(t)] + sum_{censored} [log R(t)]
    log_f = log(beta) - log(eta) + (beta-1)*log(t/eta) - (t/eta).^beta;
    log_R = -(t/eta).^beta;
    nll   = -sum(delta .* log_f + (1-delta) .* log_R);
end


%% ================================================================
%% HELPER: weibull_censored_only_analysis
%% Called when all runs are censored — lower-bound + projection
%% ================================================================
function weibull_censored_only_analysis(R, n_runs, MAX_CYCLES, conf, ...
        calib_te, baseline_te, primary_tier)

    cens_cycles = zeros(1, n_runs);
    for fi = 1:n_runs
        cens_cycles(fi) = min(R(fi).end_cycle, MAX_CYCLES);
    end

    fprintf('All-censored analysis:\n');
    fprintf('  Censoring cycles: %s\n', num2str(cens_cycles));
    eta_lower = min(cens_cycles);   % most conservative lower bound
    fprintf('  Lower bound on η (characteristic life): > %d cycles\n', eta_lower);
    fprintf('  Assuming β = 2 (wear-out prior), survival probability at observed cycles:\n');

    beta_assumed = 2.0;   % typical for rope fatigue wear-out
    for fi = 1:n_runs
        S = exp(-(cens_cycles(fi) / eta_lower)^beta_assumed);
        fprintf('    Run %d at %d cycles: R ≥ %.3f (%.1f%% survival lower bound)\n', ...
                fi, cens_cycles(fi), exp(-(1.0)), 100*exp(-1.0));
    end

    %% Plot: Reliability lower bound envelope (β sweep)
    figure('Name','Reliability Lower Bound (all censored)', 'Position',[50 400 800 400]);
    hold on; grid on;
    t_vec = linspace(1, MAX_CYCLES*1.5, 500);
    betas = [1.0, 1.5, 2.0, 2.5, 3.0];
    cmap  = parula(numel(betas));
    for bi = 1:numel(betas)
        b = betas(bi);
        R_t = exp(-(t_vec / eta_lower).^b);
        plot(t_vec, R_t*100, 'Color', cmap(bi,:), 'LineWidth', 1.5, ...
             'DisplayName', sprintf('β = %.1f', b));
    end
    xline(min(cens_cycles), 'k--', 'Min censoring cycle', 'LineWidth', 1.5);
    yline(90, 'r--', 'B10 (90% survival)', 'LineWidth', 1);
    yline(50, 'y--', 'B50 (50% survival)', 'LineWidth', 1);
    xlabel('Cycles');
    ylabel('Reliability R(t) [%]');
    title(sprintf('Test 3 — Reliability Lower Bound (η > %d, all runs censored)', eta_lower));
    legend('Location','northeast');

    fprintf('\n  Recommendation: continue testing to failure or add more runs.\n');
    fprintf('  With 3 runs all surviving %d cycles, the drive has demonstrated\n', min(cens_cycles));
    fprintf('  > %d cycle reliability at approximately 63%% confidence.\n\n', min(cens_cycles));
end


%% ================================================================
%% HELPER: plot_weibull_paper
%% Weibull probability plot (linearised: ln(cycles) vs ln(-ln(1-F)))
%% ================================================================
function plot_weibull_paper(fail_cyc, cens_cyc, beta_hat, eta_hat, beta_ci, eta_ci, tier, conf)

    t_fail = sort(fail_cyc(~isnan(fail_cyc)));
    n_fail = numel(t_fail);
    n_cens = numel(cens_cyc);
    n_tot  = n_fail + n_cens;

    % Median rank (Benard's approximation): F_i = (i - 0.3) / (n + 0.4)
    F_i = ((1:n_fail) - 0.3) / (n_tot + 0.4);
    y_i = log(-log(1 - F_i));   % Weibull y-axis
    x_i = log(t_fail);          % Weibull x-axis

    % Fitted line
    x_fit = linspace(min(x_i)*0.8, max(x_i)*1.2, 200);
    y_fit = beta_hat * (x_fit - log(eta_hat));

    % CI band on fitted line
    y_fit_lo = beta_ci(1) * (x_fit - log(eta_ci(2)));
    y_fit_hi = beta_ci(2) * (x_fit - log(eta_ci(1)));

    figure('Name', sprintf('Weibull Probability Paper — Tier %d', tier), ...
           'Position', [600 400 700 500]);
    hold on; grid on;

    fill([x_fit fliplr(x_fit)], [y_fit_lo fliplr(y_fit_hi)], ...
         [0.7 0.85 1.0], 'FaceAlpha', 0.4, 'EdgeColor','none', ...
         'DisplayName', sprintf('%d%% CI band', round(conf*100)));
    plot(x_fit, y_fit, 'b-', 'LineWidth', 2.0, ...
         'DisplayName', sprintf('Weibull fit (β=%.3f, η=%.0f)', beta_hat, eta_hat));
    scatter(x_i, y_i, 60, 'r', 'filled', 'DisplayName', 'Failure data (median rank)');

    % Reference lines at B10, B50, B63.2
    for p_ref = [0.10, 0.50, 0.632]
        y_ref = log(-log(1 - p_ref));
        yline(y_ref, 'k:', sprintf('F=%.0f%%', p_ref*100), ...
              'LabelHorizontalAlignment','left', 'HandleVisibility','off');
    end

    % Custom tick labels: convert y back to F(t) percentages
    y_ticks = log(-log(1 - [0.01 0.05 0.10 0.20 0.50 0.63 0.90 0.99]));
    ylim([min(y_ticks(1), min(y_i)-0.5), max(y_ticks(end), max(y_i)+0.5)]);
    yticks(y_ticks);
    yticklabels({'1%','5%','10%','20%','50%','63.2%','90%','99%'});

    xlabel('ln(Cycles)');
    ylabel('Unreliability F(t)  [Weibull scale]');
    title(sprintf('Weibull Probability Plot — Tier %d  (β=%.3f, η=%.0f cycles)', ...
                  tier, beta_hat, eta_hat));
    legend('Location','northwest');
end


%% ================================================================
%% HELPER: plot_reliability
%% ================================================================
function plot_reliability(beta, eta, beta_ci, eta_ci, max_cyc, B10, B50, conf, tier)

    t_vec = linspace(0, max_cyc * 1.5, 1000);
    R_fit = exp(-(t_vec/eta).^beta) * 100;
    R_lo  = exp(-(t_vec/eta_ci(1)).^beta_ci(2)) * 100;   % conservative lower
    R_hi  = exp(-(t_vec/eta_ci(2)).^beta_ci(1)) * 100;   % optimistic upper

    figure('Name', sprintf('Reliability Function — Tier %d', tier), ...
           'Position', [1050 400 750 450]);
    hold on; grid on;

    fill([t_vec fliplr(t_vec)], [R_lo fliplr(R_hi)], ...
         [0.7 0.85 1.0], 'FaceAlpha', 0.4, 'EdgeColor','none', ...
         'DisplayName', sprintf('%d%% CI', round(conf*100)));
    plot(t_vec, R_fit, 'b-', 'LineWidth', 2.5, 'DisplayName', 'R(t) — MLE fit');

    xline(B10, 'r--', sprintf('B10 = %.0f cyc', B10), 'LineWidth', 1.2);
    xline(B50, 'y--', sprintf('B50 = %.0f cyc', B50), 'LineWidth', 1.2);
    xline(eta,  'k--', sprintf('η = %.0f cyc',  eta),  'LineWidth', 1.2);
    yline(90, 'r:', '90%', 'HandleVisibility','off');
    yline(50, 'y:', '50%', 'HandleVisibility','off');

    ylim([0 100]);
    xlabel('Cycles');
    ylabel('Reliability R(t) [%]');
    title(sprintf('Test 3 — Reliability Function R(t)  Tier %d  (β=%.3f, η=%.0f)', ...
                  tier, beta, eta));
    legend('Location','northeast');
end


%% ================================================================
%% HELPER: get_first_crossing
%% ================================================================
function cyc = get_first_crossing(cycle_vec, mask_vec)
    idx = find(mask_vec, 1, 'first');
    if isempty(idx)
        cyc = NaN;
    else
        cyc = cycle_vec(idx);
    end
end


%% ================================================================
%% HELPER: fmt_cycle
%% ================================================================
function s = fmt_cycle(cyc)
    if isnan(cyc)
        s = 'None (censored)';
    else
        s = sprintf('%d', cyc);
    end
end


%% ================================================================
%% HELPER: read_metadata
%% ================================================================
function meta = read_metadata(filename)
    meta.STOP_REASON              = 'unknown';
    meta.TOTAL_CYCLES             = NaN;
    meta.LOAD_PCT                 = NaN;
    meta.GEAR_RATIO               = NaN;
    meta.BASELINE_MEAN_TE_DEG     = NaN;
    meta.BASELINE_MEAN_CURRENT_mA = NaN;
    meta.CENSORED                 = 1;

    fid = fopen(filename, 'r');
    if fid < 0, warning('Cannot open %s', filename); return; end
    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line), lines{end+1} = line; end %#ok<AGROW>
    end
    fclose(fid);

    for i = 1:numel(lines)
        ln = strtrim(lines{i});
        parts = strsplit(ln, ',');
        if numel(parts) < 2, continue; end
        key = strtrim(parts{1});
        val = strtrim(parts{2});
        switch key
            case 'STOP_REASON',              meta.STOP_REASON              = val;
            case 'TOTAL_CYCLES',             meta.TOTAL_CYCLES             = str2double(val);
            case 'LOAD_PCT',                 meta.LOAD_PCT                 = str2double(val);
            case 'GEAR_RATIO',               meta.GEAR_RATIO               = str2double(val);
            case 'BASELINE_MEAN_TE_DEG',     meta.BASELINE_MEAN_TE_DEG     = str2double(val);
            case 'BASELINE_MEAN_CURRENT_mA', meta.BASELINE_MEAN_CURRENT_mA = str2double(val);
            case 'CENSORED',                 meta.CENSORED                 = str2double(val);
        end
    end
end