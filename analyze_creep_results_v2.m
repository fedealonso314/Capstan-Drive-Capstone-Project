function analyze_creep_results_v2()
% =========================================================================
%  CREEP / STATIC LOAD TEST ANALYSIS — Test 2 (creep_static_load_test_v1.m output)
%  Capstan Drive Reliability Project
%
%  Reads:  test2_creep_*.csv  (produced by creep_static_load_test_v1.m)
%  Computes:
%
%  1. Per-rep drift trajectory (encoder position vs time)
%  2. Snapshot values at 1, 5, 15, 30 min (or available time)
%  3. Mean ± SD drift across reps at each snapshot
%  4. Transmission error statistics per rep
%  5. Motor current statistics per rep
%  6. Drift rate (deg/min) — linear fit over time for each rep
%  7. Failure tier assessment at each snapshot
%  8. Creep model fit: logarithmic (viscoelastic) vs linear
%  9. Statistical comparison across reps (t-test if 3 reps)
% 10. Comprehensive plots (5 figures)
% 11. Logbook-ready summary table
%
%  HOW TO USE:
%    1. Place this file in the same folder as your test2_creep_*.csv
%    2. Run: analyze_creep_results()
%    3. Script auto-selects most recent CSV, or set CSV_FILENAME manually
% =========================================================================

clc;
clear;
close all;

%% ================================================================
%% TUNING
%% ================================================================

CSV_FILENAME        = 'test2_creep_2026-06-04_21-20-56.csv';   % Leave '' to auto-select, or specify manually

% Snapshot times to evaluate (seconds)
SNAPSHOT_TIMES_SEC  = [300, 900, 1800, 3600];
SNAPSHOT_LABELS     = {'5 min', '15 min', '30 min', '60 min'};

% Failure tier thresholds (methodology doc §2)
TIER1_DRIFT_DEG     = 1.0;
TIER2_DRIFT_DEG     = 3.0;
TIER3_DRIFT_DEG     = 15.0;

% Baseline TE reference from Test 0 (update after running analyze_baseline_results)
BASELINE_MEAN_TE    = -0.3407;   % deg — from Test 0 analysis

%% ================================================================
%% 1. LOAD DATA
%% ================================================================

if isempty(CSV_FILENAME)
    files = dir('test2_creep_*.csv');
    if isempty(files)
        files = dir('/mnt/user-data/uploads/test2_creep*.csv');
    end
    if isempty(files)
        error('No test2_creep_*.csv found. Set CSV_FILENAME manually.');
    end
    [~, idx] = max([files.datenum]);
    CSV_FILENAME = fullfile(files(idx).folder, files(idx).name);
    fprintf('Auto-selected: %s\n\n', CSV_FILENAME);
end

metadata = read_metadata(CSV_FILENAME);
gear_ratio   = metadata.GEAR_RATIO;
output_range = metadata.OUTPUT_RANGE_DEG;

fprintf('Metadata:\n');
fprintf('  Gear ratio       : %.6f\n', gear_ratio);
fprintf('  Output range     : %.4f deg\n', output_range);
fprintf('  Hold position    : %.1f deg (midpoint)\n\n', metadata.HOLD_POSITION_DEG);

%% Read data rows
raw = readtable(CSV_FILENAME, 'VariableNamingRule', 'preserve');
valid = ~isnan(raw.REP) & ~isnan(raw.TIME_SEC);
raw  = raw(valid, :);

rep         = raw.REP;
time_s      = raw.TIME_SEC;
dxl_pos     = raw.DXL_POS_DEG;
enc_pos     = raw.ENC_POS_DEG;       % Drift from hold position (hold = 0 deg)
exp_out     = raw.EXP_OUTPUT_DEG;
trans_err   = raw.TRANS_ERR_DEG;
drift       = raw.DRIFT_DEG;         % Same as enc_pos when hold=0
current_mA  = raw.CURRENT_mA;
tier_logged = raw.FAILURE_TIER;

reps    = unique(rep)';
n_reps  = numel(reps);
n_total = height(raw);

fprintf('Data loaded: %d samples across %d repetitions\n\n', n_total, n_reps);

%% Check actual max duration per rep
for ri = 1:n_reps
    m = rep == reps(ri);
    fprintf('  Rep %d: %d samples, duration %.1f s (%.1f min)\n', ...
            reps(ri), sum(m), max(time_s(m)), max(time_s(m))/60);
end
fprintf('\n');

%% ================================================================
%% 2. PER-REP STATISTICS
%% ================================================================

rep_mean_drift = zeros(1, n_reps);
rep_std_drift  = zeros(1, n_reps);
rep_max_drift  = zeros(1, n_reps);
rep_mean_te    = zeros(1, n_reps);
rep_std_te     = zeros(1, n_reps);
rep_mean_curr  = zeros(1, n_reps);
rep_drift_rate = zeros(1, n_reps);   % deg/min from linear fit
rep_log_A      = zeros(1, n_reps);   % Logarithmic fit: drift = A*ln(t+1) + B
rep_log_B      = zeros(1, n_reps);
rep_log_rms    = zeros(1, n_reps);
rep_lin_rms    = zeros(1, n_reps);

for ri = 1:n_reps
    m = rep == reps(ri);
    t_r = time_s(m);
    d_r = drift(m);
    te_r = trans_err(m);
    cur_r = abs(current_mA(m));

    rep_mean_drift(ri) = mean(d_r);
    rep_std_drift(ri)  = std(d_r);
    rep_max_drift(ri)  = max(abs(d_r));
    rep_mean_te(ri)    = mean(te_r);
    rep_std_te(ri)     = std(te_r);
    rep_mean_curr(ri)  = mean(cur_r);

    % Linear drift rate (deg/min)
    if numel(t_r) > 2
        p = polyfit(t_r / 60, d_r, 1);
        rep_drift_rate(ri) = p(1);

        % Logarithmic creep model: drift = A*ln(t+1) + B
        t_log = log(t_r + 1);
        p_log = polyfit(t_log, d_r, 1);
        rep_log_A(ri) = p_log(1);
        rep_log_B(ri) = p_log(2);

        % RMS residuals for model comparison
        lin_pred = polyval(p, t_r/60);
        log_pred = p_log(1) * log(t_r + 1) + p_log(2);
        rep_lin_rms(ri) = sqrt(mean((d_r - lin_pred).^2));
        rep_log_rms(ri) = sqrt(mean((d_r - log_pred).^2));
    end
end

%% ================================================================
%% 3. SNAPSHOT ANALYSIS
%% ================================================================

% For each snapshot time: find nearest sample per rep, aggregate across reps
n_snap = numel(SNAPSHOT_TIMES_SEC);
snap_drift  = NaN(n_reps, n_snap);
snap_te     = NaN(n_reps, n_snap);
snap_curr   = NaN(n_reps, n_snap);
snap_tier   = NaN(n_reps, n_snap);

for ri = 1:n_reps
    m    = rep == reps(ri);
    t_r  = time_s(m);
    d_r  = drift(m);
    te_r = trans_err(m);
    cur_r = abs(current_mA(m));

    for si = 1:n_snap
        t_target = SNAPSHOT_TIMES_SEC(si);
        if max(t_r) < t_target - 60   % 60 s tolerance for late-arriving final sample
            % Rep ended before this snapshot — mark as unavailable
            continue;
        end
        [~, idx] = min(abs(t_r - t_target));
        snap_drift(ri, si) = d_r(idx);
        snap_te(ri, si)    = te_r(idx);
        snap_curr(ri, si)  = cur_r(idx);

        d_abs = abs(d_r(idx));
        if d_abs >= TIER3_DRIFT_DEG
            snap_tier(ri, si) = 3;
        elseif d_abs >= TIER2_DRIFT_DEG
            snap_tier(ri, si) = 2;
        elseif d_abs >= TIER1_DRIFT_DEG
            snap_tier(ri, si) = 1;
        else
            snap_tier(ri, si) = 0;
        end
    end
end

%% ================================================================
%% 4. PRINT RESULTS
%% ================================================================

fprintf('==========================================================\n');
fprintf('  PER-REP STATISTICS\n');
fprintf('  %-5s | %-10s | %-10s | %-10s | %-10s | %-10s | %-12s\n', ...
        'Rep', 'Mean Drift', 'Max |Drift|', 'Mean TE', 'Std TE', 'Mean |I|', 'Drift Rate');
fprintf('  %s\n', repmat('-', 1, 80));
for ri = 1:n_reps
    fprintf('  %-5d | %+9.4f | %10.4f | %+9.4f | %9.4f | %9.1f | %+.4f deg/min\n', ...
            reps(ri), rep_mean_drift(ri), rep_max_drift(ri), ...
            rep_mean_te(ri), rep_std_te(ri), rep_mean_curr(ri), rep_drift_rate(ri));
end
fprintf('==========================================================\n\n');

fprintf('CREEP MODEL FIT (per rep):\n');
fprintf('  %-5s | %-25s | %-12s | %-25s | %-12s | %-12s\n', ...
        'Rep', 'Log fit: A*ln(t+1)+B', 'Log RMS', 'Linear fit rate', 'Lin RMS', 'Better fit');
fprintf('  %s\n', repmat('-', 1, 100));
for ri = 1:n_reps
    if rep_log_rms(ri) < rep_lin_rms(ri)
        better = 'Logarithmic';
    else
        better = 'Linear';
    end
    fprintf('  %-5d | A=%.4f B=%.4f          | %.4f deg    | %.4f deg/min          | %.4f deg  | %s\n', ...
            reps(ri), rep_log_A(ri), rep_log_B(ri), rep_log_rms(ri), ...
            rep_drift_rate(ri), rep_lin_rms(ri), better);
end
fprintf('\n');

fprintf('SNAPSHOT ANALYSIS (drift from hold position):\n');
fprintf('  %-12s | %-10s | %-10s | %-10s | %-10s | %-10s\n', ...
        'Time', 'Rep 1', 'Rep 2', 'Rep 3', 'Mean ± SD', 'Tier');
fprintf('  %s\n', repmat('-', 1, 72));
for si = 1:n_snap
    vals = snap_drift(:, si);
    valid_vals = vals(~isnan(vals));
    if isempty(valid_vals)
        mean_str = 'N/A';
    elseif numel(valid_vals) == 1
        mean_str = sprintf('%+.4f (n=1)', valid_vals(1));
    else
        mean_str = sprintf('%+.4f ± %.4f', mean(valid_vals), std(valid_vals));
    end
    tier_max = max(snap_tier(:, si), [], 'omitnan');
    tier_str = {'Tier 0','Tier 1','Tier 2','Tier 3'};

    rep_vals_str = '';
    for ri = 1:n_reps
        if isnan(snap_drift(ri, si))
            rep_vals_str = [rep_vals_str, sprintf('%-10s | ', 'N/A')]; %#ok<AGROW>
        else
            rep_vals_str = [rep_vals_str, sprintf('%+9.4f | ', snap_drift(ri, si))]; %#ok<AGROW>
        end
    end

    fprintf('  %-12s | %s%-22s | %s\n', ...
            SNAPSHOT_LABELS{si}, rep_vals_str, mean_str, tier_str{min(tier_max+1,4)});
end
fprintf('\n');

%% t-test across reps at final available snapshot (if 3 reps with data)
final_snap = n_snap;
while final_snap > 1 && sum(~isnan(snap_drift(:, final_snap))) < 2
    final_snap = final_snap - 1;
end
valid_final = snap_drift(~isnan(snap_drift(:, final_snap)), final_snap);
if numel(valid_final) >= 2
    [~, p_val, ci_val] = ttest(valid_final);
    fprintf('ONE-SAMPLE t-TEST at %s (drift vs 0):\n', SNAPSHOT_LABELS{final_snap});
    fprintf('  n=%d reps, mean drift = %+.4f deg\n', numel(valid_final), mean(valid_final));
    fprintf('  p-value = %.4f  (95%% CI: [%+.4f, %+.4f] deg)\n\n', p_val, ci_val(1), ci_val(2));
end

%% ================================================================
%% FORMATTED SNAPSHOT SUMMARY TABLE (matches paper table layout)
%% ================================================================

% Column widths for fixed-width console table
COL_TIME  = 12;
COL_REP   = 11;
COL_MEAN  = 18;
divider   = repmat('-', 1, COL_TIME + n_reps*COL_REP + COL_MEAN + 4);

fprintf('%s\n', divider);

% Header row
hdr = sprintf('%-*s', COL_TIME, 'Time (min)');
for ri = 1:n_reps
    hdr = [hdr, sprintf('  Rep %d (°)%*s', reps(ri), COL_REP - 10, '')]; %#ok<AGROW>
end
hdr = [hdr, sprintf('  Mean ± SD (°)')];
fprintf('%s\n', hdr);
fprintf('%s\n', divider);

% Data rows
for si = 1:n_snap
    % Time label — strip ' min' and show just the number
    t_min_val = SNAPSHOT_TIMES_SEC(si) / 60;
    row = sprintf('%-*g', COL_TIME, t_min_val);

    vals = snap_drift(:, si);
    for ri = 1:n_reps
        if isnan(vals(ri))
            row = [row, sprintf('  %-*s', COL_REP-2, 'N/A')]; %#ok<AGROW>
        else
            row = [row, sprintf('  %*.3f    ', COL_REP-6, vals(ri))]; %#ok<AGROW>
        end
    end

    valid_vals = vals(~isnan(vals));
    if isempty(valid_vals)
        mean_sd_str = 'N/A';
    elseif numel(valid_vals) == 1
        mean_sd_str = sprintf('%.3f (n=1)', valid_vals(1));
    else
        mean_sd_str = sprintf('%.3f ± %.3f', mean(valid_vals), std(valid_vals));
    end
    row = [row, sprintf('  %s', mean_sd_str)]; %#ok<AGROW>
    fprintf('%s\n', row);
end
fprintf('%s\n\n', divider);

%% ================================================================
%% 5. SAVE ANALYSIS CSV
%% ================================================================

% Snapshot summary table
snap_mean = zeros(1, n_snap);
snap_sd   = zeros(1, n_snap);
for si = 1:n_snap
    v = snap_drift(~isnan(snap_drift(:, si)), si);
    if ~isempty(v)
        snap_mean(si) = mean(v);
        snap_sd(si)   = std(v);
    else
        snap_mean(si) = NaN;
        snap_sd(si)   = NaN;
    end
end

T_snap = table(SNAPSHOT_TIMES_SEC', SNAPSHOT_LABELS', snap_mean', snap_sd', ...
    'VariableNames', {'TIME_SEC','LABEL','MEAN_DRIFT_DEG','SD_DRIFT_DEG'});

out_filename = sprintf('test2_analysis_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
writetable(T_snap, out_filename);

fid = fopen(out_filename, 'a');
fprintf(fid, '\nPER_REP_SUMMARY\n');
fprintf(fid, 'REP,MEAN_DRIFT_DEG,MAX_DRIFT_DEG,MEAN_TE_DEG,STD_TE_DEG,MEAN_CURR_mA,DRIFT_RATE_DEG_PER_MIN,LOG_A,LOG_B,LOG_RMS,LIN_RMS\n');
for ri = 1:n_reps
    fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            reps(ri), rep_mean_drift(ri), rep_max_drift(ri), rep_mean_te(ri), ...
            rep_std_te(ri), rep_mean_curr(ri), rep_drift_rate(ri), ...
            rep_log_A(ri), rep_log_B(ri), rep_log_rms(ri), rep_lin_rms(ri));
end
fprintf(fid, '\nMETADATA_ECHO\n');
fprintf(fid, 'GEAR_RATIO,%.6f\n', gear_ratio);
fprintf(fid, 'OUTPUT_RANGE_DEG,%.4f\n', output_range);
fprintf(fid, 'BASELINE_MEAN_TE_DEG_REF,%.4f\n', BASELINE_MEAN_TE);
fclose(fid);

fprintf('Analysis saved to: %s\n\n', out_filename);

%% ================================================================
%% 6. PLOTS
%% ================================================================

colors = lines(n_reps);
tier_colors = [0.2 0.7 0.2; 1.0 0.8 0.0; 1.0 0.5 0.0; 0.8 0.1 0.1];

%% ---- Figure 1: Drift vs Time — all reps overlaid ----
figure('Name','Fig 1 — Drift vs Time', 'Position',[50 600 1000 380]);
hold on; grid on;
for ri = 1:n_reps
    m = rep == reps(ri);
    t_min = time_s(m) / 60;
    plot(t_min, drift(m), 'Color', colors(ri,:), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Rep %d', reps(ri)));

    % Overlay log model
    t_fit = linspace(min(time_s(m)), max(time_s(m)), 300);
    d_fit = rep_log_A(ri) * log(t_fit + 1) + rep_log_B(ri);
    plot(t_fit/60, d_fit, '--', 'Color', colors(ri,:)*0.6, 'LineWidth', 1.0, ...
         'HandleVisibility','off');
end
% Add snapshot vertical lines
for si = 1:n_snap
    if SNAPSHOT_TIMES_SEC(si)/60 <= max(time_s)/60
        xline(SNAPSHOT_TIMES_SEC(si)/60, 'k:', SNAPSHOT_LABELS{si}, ...
              'LabelHorizontalAlignment','right', 'HandleVisibility','off');
    end
end
yline( TIER2_DRIFT_DEG, 'r--', 'Tier 2 (+3 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left');
yline(-TIER2_DRIFT_DEG, 'r--', '',                 'LineWidth', 1, 'HandleVisibility','off');
yline( TIER1_DRIFT_DEG, 'y--', 'Tier 1 (+1 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left');
yline(-TIER1_DRIFT_DEG, 'y--', '',                 'LineWidth', 1, 'HandleVisibility','off');
yline(0, 'k-', 'Hold position', 'LineWidth', 0.8);
xlabel('Time [min]');
ylabel('Output Drift from Hold Position [deg]');
title('Test 2 — Creep: Output Drift vs Time (Hold at 0 deg, Dead Load)');
legend('Location','southeast');
annotation('textbox', [0.15 0.75 0.25 0.10], 'String', ...
    'Dashed = log model fit', 'EdgeColor','none', 'FontSize',8, 'Color',[0.4 0.4 0.4]);

%% ---- Figure 2: Snapshot bar chart — drift at 1, 5, 15, 30 min ----
figure('Name','Fig 2 — Snapshot Drift', 'Position',[50 160 700 380]);
hold on; grid on;

bar_x = 1:n_snap;
bar_data = zeros(n_reps, n_snap);
for ri = 1:n_reps
    bar_data(ri, :) = snap_drift(ri, :);
end
b = bar(bar_x, bar_data', 'grouped');
for ri = 1:n_reps
    b(ri).FaceColor = colors(ri,:);
    b(ri).DisplayName = sprintf('Rep %d', reps(ri));
end

% Mean line
valid_snaps = ~all(isnan(snap_drift));
if any(valid_snaps)
    means = snap_mean(valid_snaps);
    plot(bar_x(valid_snaps), means, 'k-o', 'LineWidth', 2, ...
         'MarkerSize', 6, 'DisplayName', 'Mean across reps');
end
yline( TIER1_DRIFT_DEG, 'y--', 'Tier 1', 'LineWidth', 1.2);
yline(-TIER1_DRIFT_DEG, 'y--', '',        'LineWidth', 1.2, 'HandleVisibility','off');
xticks(bar_x);
xticklabels(SNAPSHOT_LABELS);
xlabel('Snapshot Time');
ylabel('Drift [deg]');
title('Test 2 — Drift at Snapshot Times per Rep');
legend('Location','best');

%% ---- Figure 3: Transmission Error vs Time — all reps ----
figure('Name','Fig 3 — TE vs Time', 'Position',[1050 600 900 380]);
hold on; grid on;
for ri = 1:n_reps
    m = rep == reps(ri);
    plot(time_s(m)/60, trans_err(m), 'Color', colors(ri,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Rep %d', reps(ri)));
end
yline(BASELINE_MEAN_TE,              'b-',  'Baseline TE', 'LineWidth', 1.5);
yline(BASELINE_MEAN_TE + 1.0,        'y--', 'Baseline ±1 deg', 'LineWidth', 1);
yline(BASELINE_MEAN_TE - 1.0,        'y--', '',               'LineWidth', 1, 'HandleVisibility','off');
xlabel('Time [min]');
ylabel('Transmission Error [deg]');
title('Test 2 — Transmission Error vs Time');
legend('Location','best');

%% ---- Figure 4: Motor current vs time ----
figure('Name','Fig 4 — Current vs Time', 'Position',[1050 160 900 380]);
hold on; grid on;
for ri = 1:n_reps
    m = rep == reps(ri);
    plot(time_s(m)/60, abs(current_mA(m)), 'Color', colors(ri,:), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Rep %d', reps(ri)));
end
xlabel('Time [min]');
ylabel('|Current| [mA]');
title('Test 2 — Motor Holding Current vs Time');
legend('Location','best');

%% ---- Figure 5: Logarithmic creep model overlay (all reps) ----
figure('Name','Fig 5 — Creep Model Fit', 'Position',[550 380 800 400]);
hold on; grid on;
for ri = 1:n_reps
    m = rep == reps(ri);
    t_r = time_s(m);
    d_r = drift(m);
    plot(t_r/60, d_r, 'o', 'Color', colors(ri,:), 'MarkerSize', 2, ...
         'HandleVisibility','off');
    t_fit = linspace(0, max(t_r), 400)';
    d_log = rep_log_A(ri) * log(t_fit + 1) + rep_log_B(ri);
    d_lin = rep_drift_rate(ri) * t_fit/60 + rep_log_B(ri);
    plot(t_fit/60, d_log, '-',  'Color', colors(ri,:), 'LineWidth', 2.0, ...
         'DisplayName', sprintf('Rep %d Log (RMS=%.3f)', ri, rep_log_rms(ri)));
    plot(t_fit/60, d_lin, '--', 'Color', colors(ri,:)*0.6, 'LineWidth', 1.2, ...
         'DisplayName', sprintf('Rep %d Linear (RMS=%.3f)', ri, rep_lin_rms(ri)));
end
yline( TIER1_DRIFT_DEG, 'y--', 'Tier 1', 'LineWidth', 1);
yline(-TIER1_DRIFT_DEG, 'y--', '',       'LineWidth', 1, 'HandleVisibility','off');
xlabel('Time [min]');
ylabel('Drift [deg]');
title('Test 2 — Creep Model: Logarithmic (solid) vs Linear (dashed)');
legend('Location','best', 'NumColumns', 2);

fprintf('All plots generated.\n\n');

%% ================================================================
%% LOGBOOK SUMMARY
%% ================================================================

fprintf('==========================================================\n');
fprintf('  LOGBOOK ENTRY — Test 2 Creep / Static Load\n');
fprintf('==========================================================\n');
fprintf('  Date/time         : %s\n', datestr(now));
fprintf('  Source file       : %s\n', CSV_FILENAME);
fprintf('  Gear ratio        : %.6f\n', gear_ratio);
fprintf('  Hold position     : 0 deg (midpoint)\n');
fprintf('  Reps completed    : %d of 3\n', n_reps);
fprintf('  -------------------------------------------------------\n');
fprintf('  DRIFT SNAPSHOTS (mean ± SD across reps)\n');
for si = 1:n_snap
    v = snap_drift(~isnan(snap_drift(:,si)), si);
    if isempty(v)
        fprintf('  %s  : no data (rep ended before this time)\n', SNAPSHOT_LABELS{si});
    elseif numel(v) == 1
        fprintf('  %s  : %+.4f deg  (1 rep only)\n', SNAPSHOT_LABELS{si}, v(1));
    else
        fprintf('  %s  : %+.4f ± %.4f deg  (n=%d)\n', SNAPSHOT_LABELS{si}, mean(v), std(v), numel(v));
    end
end
fprintf('  -------------------------------------------------------\n');
fprintf('  CREEP CHARACTERIZATION\n');
for ri = 1:n_reps
    fprintf('  Rep %d drift rate  : %+.4f deg/min (linear)\n', reps(ri), rep_drift_rate(ri));
    fprintf('  Rep %d log model   : %.4f*ln(t+1) + %.4f  (RMS=%.4f deg)\n', ...
            reps(ri), rep_log_A(ri), rep_log_B(ri), rep_log_rms(ri));
end
fprintf('  -------------------------------------------------------\n');
fprintf('  FAILURE ASSESSMENT\n');
for si = 1:n_snap
    v = snap_drift(~isnan(snap_drift(:,si)), si);
    if ~isempty(v)
        t_max = max(snap_tier(~isnan(snap_drift(:,si)), si));
        tier_str = {'Tier 0 (Nominal)','Tier 1 (Minimal)','Tier 2 (Intermediate)','Tier 3 (Critical)'};
        fprintf('  At %s: max tier = %s\n', SNAPSHOT_LABELS{si}, tier_str{min(t_max+1,4)});
    end
end
fprintf('==========================================================\n');

end  %% ---- END MAIN ----


%% ================================================================
%% HELPER: read_metadata
%% ================================================================
function meta = read_metadata(filename)
    meta.GEAR_RATIO         = NaN;
    meta.INPUT_RANGE_DEG    = NaN;
    meta.OUTPUT_RANGE_DEG   = NaN;
    meta.HOLD_POSITION_DEG  = 0;

    fid = fopen(filename, 'r');
    if fid < 0, error('Cannot open: %s', filename); end
    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line), lines{end+1} = line; end %#ok<AGROW>
    end
    fclose(fid);

    for i = 1:numel(lines)
        ln = strtrim(lines{i});
        if startsWith(ln, 'GEAR_RATIO,')
            parts = strsplit(ln, ','); meta.GEAR_RATIO = str2double(parts{2});
        elseif startsWith(ln, 'INPUT_RANGE_DEG,')
            parts = strsplit(ln, ','); meta.INPUT_RANGE_DEG = str2double(parts{2});
        elseif startsWith(ln, 'OUTPUT_RANGE_DEG,')
            parts = strsplit(ln, ','); meta.OUTPUT_RANGE_DEG = str2double(parts{2});
        elseif startsWith(ln, 'HOLD_POSITION_DEG,')
            parts = strsplit(ln, ','); meta.HOLD_POSITION_DEG = str2double(parts{2});
        end
    end
end