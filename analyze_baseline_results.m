function analyze_baseline_results()
% =========================================================================
%  BASELINE TEST ANALYSIS — Test 0 (baseline_test_v2.m output)
%  Capstan Drive Reliability Project
%
%  Reads:  test0_baseline_*.csv  (produced by baseline_test_v2.m)
%  Computes all metrics required for the methodology document:
%
%  1. Gear ratio (from metadata — derived from physical limit search)
%  2. Transmission Error (TE) statistics per cycle and global
%  3. Motor current statistics per cycle and global
%  4. Position tracking accuracy (expected vs actual output)
%  5. Baseline reference values (mean TE, mean current at cycle 0)
%     — These are the reference values used by ALL subsequent tests
%  6. Failure tier assessment across all samples
%  7. Hysteresis: TE difference between CW and CCW legs within each cycle
%  8. Degradation check: is TE changing across the 20 cycles?
%  9. Comprehensive plots (6 figures)
% 10. Summary table printed to console — paste directly into logbook
%
%  HOW TO USE:
%    1. Place this file in the same folder as your test0_baseline_*.csv
%    2. Run: analyze_baseline_results()
%    3. Script auto-selects the most recent CSV if multiple exist
%       OR you can specify a filename directly (see TUNING section below)
% =========================================================================

clc;
clear;
close all;

%% ================================================================
%% TUNING — edit these if needed
%% ================================================================

CSV_FILENAME    = '';       % Leave '' to auto-select most recent test0_baseline_*.csv
                            % Or set explicitly: 'test0_baseline_2025-04-10_09-30-00.csv'

% Failure tier thresholds (from methodology doc §2)
TIER1_TE_DEG   = 1.0;      % Minimal: TE drift > ±1 deg above baseline
TIER2_TE_DEG   = 3.0;      % Intermediate: mean TE > ±3 deg over 10 cycles
TIER3_TE_DEG   = 15.0;     % Critical: catastrophic slip

TIER1_CURR_PCT_LOW  = 0.05;   % Minimal: current 5% above baseline
TIER1_CURR_PCT_HIGH = 0.10;   % Minimal upper edge
TIER2_CURR_PCT      = 0.20;   % Intermediate: current > 20% above baseline

% Hysteresis analysis: minimum absolute encoder position to include
% (excludes the reversal zone near zero where direction transitions)
HYSTERESIS_DEADBAND_DEG = 2.0;

%% ================================================================
%% 1. LOAD DATA
%% ================================================================

if isempty(CSV_FILENAME)
    files = dir('test0_baseline_*.csv');
    if isempty(files)
        % Also try the uploads path (for running inside Claude environment)
        files = dir('/mnt/user-data/uploads/test0_baseline*.csv');
    end
    if isempty(files)
        error('No test0_baseline_*.csv found in current folder. Set CSV_FILENAME manually.');
    end
    [~, idx] = max([files.datenum]);
    CSV_FILENAME = fullfile(files(idx).folder, files(idx).name);
    fprintf('Auto-selected: %s\n\n', CSV_FILENAME);
end

%% Read metadata from bottom of file first
metadata = read_metadata(CSV_FILENAME);
gear_ratio_meta = metadata.GEAR_RATIO;
input_range_deg = metadata.INPUT_RANGE_DEG;
output_range_deg = metadata.OUTPUT_RANGE_DEG;

fprintf('Metadata from file:\n');
fprintf('  Gear ratio (physical limit):  %.6f\n', gear_ratio_meta);
fprintf('  Input range (Dynamixel):      %.4f deg\n', input_range_deg);
fprintf('  Output range (Encoder):       %.4f deg\n\n', output_range_deg);

%% Read data rows (stop before METADATA line)
raw = readtable(CSV_FILENAME, 'VariableNamingRule', 'preserve');

% Find where metadata starts (empty cycle column or non-numeric)
valid_rows = ~isnan(raw.CYCLE) & ~isnan(raw.TIME);
raw = raw(valid_rows, :);

cycle       = raw.CYCLE;
time_s      = raw.TIME;
exp_out_deg = raw.EXPECTED_OUTPUT_DEG;   % = DXL_pos / gear_ratio (expected encoder output)
dxl_pos_deg = raw.DXL_POS_DEG;          % Dynamixel input position (zeroed)
enc_pos_deg = raw.ENC_POS_DEG;          % Encoder actual output (zeroed)
trans_err   = raw.TRANS_ERR_DEG;        % TE = expected_output - enc_pos
current_mA  = raw.("CURRENT_mA");

n_total  = height(raw);
n_cycles = max(cycle);

fprintf('Data loaded: %d samples across %d cycles\n\n', n_total, n_cycles);

%% ================================================================
%% 2. PER-CYCLE STATISTICS
%% ================================================================

cycle_ids = unique(cycle)';

cyc_mean_te   = zeros(1, n_cycles);
cyc_std_te    = zeros(1, n_cycles);
cyc_min_te    = zeros(1, n_cycles);
cyc_max_te    = zeros(1, n_cycles);
cyc_p2p_te    = zeros(1, n_cycles);   % Peak-to-peak TE
cyc_rms_te    = zeros(1, n_cycles);   % RMS TE
cyc_mean_curr = zeros(1, n_cycles);
cyc_std_curr  = zeros(1, n_cycles);
cyc_mean_enc  = zeros(1, n_cycles);
cyc_samples   = zeros(1, n_cycles);

for ci = 1:n_cycles
    mask = cycle == ci;
    te_c   = trans_err(mask);
    cur_c  = abs(current_mA(mask));

    cyc_mean_te(ci)   = mean(te_c);
    cyc_std_te(ci)    = std(te_c);
    cyc_min_te(ci)    = min(te_c);
    cyc_max_te(ci)    = max(te_c);
    cyc_p2p_te(ci)    = max(te_c) - min(te_c);
    cyc_rms_te(ci)    = sqrt(mean(te_c.^2));
    cyc_mean_curr(ci) = mean(cur_c);
    cyc_std_curr(ci)  = std(cur_c);
    cyc_mean_enc(ci)  = mean(abs(enc_pos_deg(mask)));
    cyc_samples(ci)   = sum(mask);
end

%% ================================================================
%% 3. GLOBAL (ALL-CYCLE) STATISTICS
%% ================================================================

global_mean_te   = mean(trans_err);
global_std_te    = std(trans_err);
global_rms_te    = sqrt(mean(trans_err.^2));
global_p2p_te    = max(trans_err) - min(trans_err);
global_mean_curr = mean(abs(current_mA));
global_std_curr  = std(abs(current_mA));
global_max_curr  = max(abs(current_mA));

%% ================================================================
%% 4. BASELINE REFERENCE VALUES
%%    (Used by ALL subsequent tests as the cycle-0 reference)
%% ================================================================

% Use mean of ALL 20 cycles as the baseline (stable measurement)
baseline_mean_te      = global_mean_te;
baseline_mean_current = global_mean_curr;

% Confidence interval on baseline (95%, t-distribution)
n_cyc = n_cycles;
t_crit = tinv(0.975, n_cyc - 1);
ci_te_half   = t_crit * std(cyc_mean_te) / sqrt(n_cyc);
ci_curr_half = t_crit * std(cyc_mean_curr) / sqrt(n_cyc);

fprintf('==========================================================\n');
fprintf('  BASELINE REFERENCE VALUES (enter these in logbook)\n');
fprintf('==========================================================\n');
fprintf('  Mean TE            = %+.4f ± %.4f deg  (95%% CI: ±%.4f deg)\n', ...
        baseline_mean_te, std(cyc_mean_te), ci_te_half);
fprintf('  Mean |current|     = %.1f ± %.1f mA     (95%% CI: ±%.1f mA)\n', ...
        baseline_mean_current, std(cyc_mean_curr), ci_curr_half);
fprintf('  RMS TE             = %.4f deg\n', global_rms_te);
fprintf('  Peak-to-peak TE    = %.4f deg\n', global_p2p_te);
fprintf('  Gear ratio (meta)  = %.6f\n', gear_ratio_meta);
fprintf('  Input range        = %.4f deg (Dynamixel)\n', input_range_deg);
fprintf('  Output range       = %.4f deg (Encoder)\n', output_range_deg);
fprintf('==========================================================\n\n');

%% ================================================================
%% 5. FAILURE TIER ASSESSMENT (sample-by-sample)
%% ================================================================

tier = zeros(n_total, 1);

% Tier 3 — catastrophic slip (single sample)
tier(abs(trans_err) > TIER3_TE_DEG) = 3;

% Tier 2 — TE > ±3 deg OR current > 20% above baseline (not already tier 3)
tier2_te   = abs(trans_err) > TIER2_TE_DEG & tier < 3;
tier2_curr = abs(current_mA) > baseline_mean_current * (1 + TIER2_CURR_PCT) & tier < 3;
tier(tier2_te | tier2_curr) = max(tier(tier2_te | tier2_curr), 2);

% Tier 1 — TE drift > ±1 deg above baseline OR current 5–10% above baseline
te_drift   = abs(trans_err - baseline_mean_te);
curr_rise  = (abs(current_mA) - baseline_mean_current) / baseline_mean_current;
tier1_te   = te_drift > TIER1_TE_DEG & tier < 2;
tier1_curr = curr_rise >= TIER1_CURR_PCT_LOW & curr_rise <= TIER1_CURR_PCT_HIGH & tier < 2;
tier(tier1_te | tier1_curr) = max(tier(tier1_te | tier1_curr), 1);

n_tier0 = sum(tier == 0);
n_tier1 = sum(tier == 1);
n_tier2 = sum(tier == 2);
n_tier3 = sum(tier == 3);

fprintf('FAILURE TIER DISTRIBUTION (all %d samples):\n', n_total);
fprintf('  Tier 0 (Nominal)       : %4d  (%.1f%%)\n', n_tier0, 100*n_tier0/n_total);
fprintf('  Tier 1 (Minimal)       : %4d  (%.1f%%)\n', n_tier1, 100*n_tier1/n_total);
fprintf('  Tier 2 (Intermediate)  : %4d  (%.1f%%)\n', n_tier2, 100*n_tier2/n_total);
fprintf('  Tier 3 (Critical)      : %4d  (%.1f%%)\n', n_tier3, 100*n_tier3/n_total);
fprintf('\n');

if n_tier3 > 0
    warning('CRITICAL FAILURE events detected in baseline data!');
end

%% ================================================================
%% 6. DIRECTION ANALYSIS (CW vs CCW leg hysteresis)
%%    Direction inferred from sign of velocity (diff of enc_pos_deg)
%% ================================================================

velocity_proxy = [0; diff(enc_pos_deg)];   % positive = CW, negative = CCW
cw_mask  = velocity_proxy > 0 & abs(enc_pos_deg) > HYSTERESIS_DEADBAND_DEG;
ccw_mask = velocity_proxy < 0 & abs(enc_pos_deg) > HYSTERESIS_DEADBAND_DEG;

te_cw    = trans_err(cw_mask);
te_ccw   = trans_err(ccw_mask);

mean_te_cw  = mean(te_cw);
mean_te_ccw = mean(te_ccw);
hysteresis  = mean_te_cw - mean_te_ccw;   % Directional TE offset

fprintf('DIRECTIONAL ANALYSIS (hysteresis):\n');
fprintf('  Mean TE CW  leg : %+.4f deg  (n=%d)\n', mean_te_cw,  sum(cw_mask));
fprintf('  Mean TE CCW leg : %+.4f deg  (n=%d)\n', mean_te_ccw, sum(ccw_mask));
fprintf('  Hysteresis (CW - CCW): %+.4f deg\n\n', hysteresis);

%% ================================================================
%% 7. DEGRADATION TREND ACROSS 20 CYCLES
%%    Linear regression of cycle mean TE vs cycle number
%% ================================================================

p_te   = polyfit(cycle_ids, cyc_mean_te,   1);
p_curr = polyfit(cycle_ids, cyc_mean_curr, 1);

slope_te_per_cycle   = p_te(1);     % deg / cycle
slope_curr_per_cycle = p_curr(1);   % mA / cycle

fprintf('DEGRADATION TREND (linear regression across %d cycles):\n', n_cycles);
fprintf('  TE slope   : %+.5f deg/cycle   (%.4f deg over all cycles)\n', ...
        slope_te_per_cycle, slope_te_per_cycle * n_cycles);
fprintf('  Curr slope : %+.4f mA/cycle    (%.2f mA over all cycles)\n', ...
        slope_curr_per_cycle, slope_curr_per_cycle * n_cycles);

if abs(slope_te_per_cycle * n_cycles) < TIER1_TE_DEG
    fprintf('  --> TE trend: NEGLIGIBLE (< %.1f deg total drift) — drive stable\n\n', TIER1_TE_DEG);
else
    fprintf('  --> TE trend: NON-NEGLIGIBLE — review plots\n\n');
end

%% ================================================================
%% 8. ENCODER LINEARITY CHECK
%%    Expected output (Dxl/ratio) vs actual encoder output
%% ================================================================

% Fit: enc_pos = a * exp_out + b (should be a~1, b~0 if gear ratio is correct)
p_lin = polyfit(exp_out_deg, enc_pos_deg, 1);
lin_a = p_lin(1);
lin_b = p_lin(2);
enc_predicted = polyval(p_lin, exp_out_deg);
lin_residuals = enc_pos_deg - enc_predicted;
lin_rms_res   = sqrt(mean(lin_residuals.^2));

fprintf('LINEARITY (enc_pos vs expected_output linear fit):\n');
fprintf('  Fit: enc = %.6f * exp + %.6f\n', lin_a, lin_b);
fprintf('  Slope deviation from 1: %+.6f (%.4f%%)\n', lin_a - 1, 100*(lin_a-1));
fprintf('  Offset: %+.4f deg\n', lin_b);
fprintf('  RMS residual: %.4f deg\n\n', lin_rms_res);

% Gear ratio from data (mean of dxl_pos / enc_pos, excluding near-zero)
valid_gr = abs(enc_pos_deg) > 2.0;
gr_from_data = mean(abs(dxl_pos_deg(valid_gr)) ./ abs(enc_pos_deg(valid_gr)));
fprintf('GEAR RATIO VERIFICATION:\n');
fprintf('  From metadata (limit search):  %.6f\n', gear_ratio_meta);
fprintf('  From data (mean |Dxl|/|Enc|):  %.6f\n', gr_from_data);
fprintf('  Discrepancy:                   %+.6f (%.4f%%)\n\n', ...
        gr_from_data - gear_ratio_meta, 100*(gr_from_data - gear_ratio_meta)/gear_ratio_meta);

%% ================================================================
%% 9. PER-CYCLE SUMMARY TABLE (console output)
%% ================================================================

fprintf('==========================================================\n');
fprintf('  PER-CYCLE SUMMARY TABLE\n');
fprintf('  %-5s | %-10s | %-10s | %-10s | %-10s | %-10s | %-10s | %-8s\n', ...
        'Cycle', 'Mean TE', 'Std TE', 'RMS TE', 'P2P TE', 'Mean |I|', 'Std |I|', 'Samples');
fprintf('  %s\n', repmat('-', 1, 82));
for ci = 1:n_cycles
    fprintf('  %-5d | %+9.4f | %9.4f | %9.4f | %9.4f | %9.1f | %9.1f | %-8d\n', ...
            ci, cyc_mean_te(ci), cyc_std_te(ci), cyc_rms_te(ci), ...
            cyc_p2p_te(ci), cyc_mean_curr(ci), cyc_std_curr(ci), cyc_samples(ci));
end
fprintf('  %s\n', repmat('-', 1, 82));
fprintf('  %-5s | %+9.4f | %9.4f | %9.4f | %9.4f | %9.1f | %9.1f |\n', ...
        'GLOBAL', global_mean_te, global_std_te, global_rms_te, ...
        global_p2p_te, global_mean_curr, global_std_curr);
fprintf('==========================================================\n\n');

%% ================================================================
%% 10. SAVE SUMMARY TO CSV
%% ================================================================

T_cyc = table(cycle_ids', cyc_mean_te', cyc_std_te', cyc_rms_te', cyc_p2p_te', ...
              cyc_mean_curr', cyc_std_curr', cyc_samples', ...
    'VariableNames', {'CYCLE','MEAN_TE_DEG','STD_TE_DEG','RMS_TE_DEG', ...
                      'P2P_TE_DEG','MEAN_CURR_mA','STD_CURR_mA','N_SAMPLES'});

out_filename = sprintf('test0_analysis_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
writetable(T_cyc, out_filename);

% Append summary metrics
fid = fopen(out_filename, 'a');
fprintf(fid, '\nSUMMARY\n');
fprintf(fid, 'BASELINE_MEAN_TE_DEG,%+.6f\n',       baseline_mean_te);
fprintf(fid, 'BASELINE_MEAN_CURRENT_mA,%.4f\n',     baseline_mean_current);
fprintf(fid, 'GLOBAL_STD_TE_DEG,%.6f\n',            global_std_te);
fprintf(fid, 'GLOBAL_RMS_TE_DEG,%.6f\n',            global_rms_te);
fprintf(fid, 'GLOBAL_P2P_TE_DEG,%.6f\n',            global_p2p_te);
fprintf(fid, 'HYSTERESIS_DEG,%+.6f\n',              hysteresis);
fprintf(fid, 'TE_SLOPE_DEG_PER_CYCLE,%+.8f\n',      slope_te_per_cycle);
fprintf(fid, 'CURRENT_SLOPE_mA_PER_CYCLE,%+.6f\n',  slope_curr_per_cycle);
fprintf(fid, 'GEAR_RATIO_FROM_METADATA,%.6f\n',     gear_ratio_meta);
fprintf(fid, 'GEAR_RATIO_FROM_DATA,%.6f\n',         gr_from_data);
fprintf(fid, 'LINEARITY_SLOPE,%.6f\n',              lin_a);
fprintf(fid, 'LINEARITY_OFFSET_DEG,%.6f\n',         lin_b);
fprintf(fid, 'LINEARITY_RMS_RESIDUAL_DEG,%.6f\n',   lin_rms_res);
fprintf(fid, 'N_SAMPLES_TIER0,%d\n',                n_tier0);
fprintf(fid, 'N_SAMPLES_TIER1,%d\n',                n_tier1);
fprintf(fid, 'N_SAMPLES_TIER2,%d\n',                n_tier2);
fprintf(fid, 'N_SAMPLES_TIER3,%d\n',                n_tier3);
fclose(fid);

fprintf('Analysis saved to: %s\n\n', out_filename);

%% ================================================================
%% 11. PLOTS
%% ================================================================

tier_colors = [0.2 0.7 0.2;   % Tier 0 — green
               1.0 0.8 0.0;   % Tier 1 — yellow
               1.0 0.5 0.0;   % Tier 2 — orange
               0.8 0.1 0.1];  % Tier 3 — red

%% ---- Figure 1: Transmission Error vs Time (all samples) ----
figure('Name','Fig 1 — TE vs Time', 'Position',[50 600 900 350]);
hold on; grid on;

% Colour points by tier
for t_val = 0:3
    m = tier == t_val;
    if any(m)
        scatter(time_s(m), trans_err(m), 4, tier_colors(t_val+1,:), 'filled', ...
                'DisplayName', sprintf('Tier %d', t_val));
    end
end
yline( TIER2_TE_DEG,  'r--', 'Tier 2 (+3 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline(-TIER2_TE_DEG,  'r--', 'Tier 2 (-3 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline( TIER1_TE_DEG,  'y--', 'Tier 1 (+1 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline(-TIER1_TE_DEG,  'y--', 'Tier 1 (-1 deg)', 'LineWidth', 1, 'LabelHorizontalAlignment','left', 'HandleVisibility','off');
yline( baseline_mean_te, 'b-', 'Baseline mean', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Transmission Error [deg]');
title('Test 0 — Transmission Error vs Time');
legend('Location','eastoutside');

%% ---- Figure 2: Per-cycle TE mean ± 1 SD ----
figure('Name','Fig 2 — TE per Cycle', 'Position',[50 200 900 350]);
hold on; grid on;

fill([cycle_ids fliplr(cycle_ids)], ...
     [cyc_mean_te + cyc_std_te, fliplr(cyc_mean_te - cyc_std_te)], ...
     [0.7 0.7 1.0], 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', '±1 SD');
plot(cycle_ids, cyc_mean_te, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Mean TE');
trend_te = polyval(p_te, cycle_ids);
plot(cycle_ids, trend_te, 'k--', 'LineWidth', 1.0, 'DisplayName', ...
     sprintf('Trend (%.5f deg/cyc)', slope_te_per_cycle));
yline( TIER1_TE_DEG + baseline_mean_te, 'y--', 'Tier 1 upper', 'LineWidth', 1);
yline(-TIER1_TE_DEG + baseline_mean_te, 'y--', 'Tier 1 lower', 'LineWidth', 1);
yline(baseline_mean_te, 'b:', 'Baseline', 'LineWidth', 1);
xlabel('Cycle Number');
ylabel('Mean Transmission Error [deg]');
title('Test 0 — Mean TE ± 1 SD per Cycle (20 warm-up cycles)');
legend('Location','best');

%% ---- Figure 3: Motor Current per cycle ----
figure('Name','Fig 3 — Current per Cycle', 'Position',[980 600 900 350]);
hold on; grid on;

fill([cycle_ids fliplr(cycle_ids)], ...
     [cyc_mean_curr + cyc_std_curr, fliplr(cyc_mean_curr - cyc_std_curr)], ...
     [1.0 0.8 0.7], 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'DisplayName', '±1 SD');
plot(cycle_ids, cyc_mean_curr, 'r-o', 'LineWidth', 1.5, 'MarkerSize', 5, 'DisplayName', 'Mean |Current|');
trend_curr = polyval(p_curr, cycle_ids);
plot(cycle_ids, trend_curr, 'k--', 'LineWidth', 1.0, 'DisplayName', ...
     sprintf('Trend (%.3f mA/cyc)', slope_curr_per_cycle));
yline(baseline_mean_current * (1 + TIER1_CURR_PCT_LOW),  'y--', '+5% Tier 1', 'LineWidth', 1);
yline(baseline_mean_current * (1 + TIER2_CURR_PCT),       'r--', '+20% Tier 2','LineWidth', 1);
xlabel('Cycle Number');
ylabel('Mean |Current| [mA]');
title('Test 0 — Motor Current per Cycle');
legend('Location','best');

%% ---- Figure 4: Expected vs Actual Output Position ----
figure('Name','Fig 4 — Position Tracking', 'Position',[980 200 900 350]);
hold on; grid on;

plot(time_s, exp_out_deg, 'b-',  'LineWidth', 0.8, 'DisplayName', 'Expected output (Dxl/ratio)');
plot(time_s, enc_pos_deg, 'r--', 'LineWidth', 0.8, 'DisplayName', 'Encoder (actual output)');
xlabel('Time [s]');
ylabel('Position [deg]');
title('Test 0 — Expected vs Actual Output Position');
legend('Location','best');

%% ---- Figure 5: TE vs Position (Lissajous / hysteresis loop) ----
figure('Name','Fig 5 — TE vs Encoder Position', 'Position',[50 50 600 400]);
hold on; grid on;

scatter(enc_pos_deg(cw_mask),  trans_err(cw_mask),  4, [0.1 0.4 0.8], 'filled', ...
        'DisplayName', 'CW leg');
scatter(enc_pos_deg(ccw_mask), trans_err(ccw_mask), 4, [0.8 0.2 0.1], 'filled', ...
        'DisplayName', 'CCW leg');
yline( TIER1_TE_DEG + baseline_mean_te, 'y--', 'Tier 1 upper', 'LineWidth', 1);
yline(-TIER1_TE_DEG + baseline_mean_te, 'y--', 'Tier 1 lower', 'LineWidth', 1);
yline(baseline_mean_te, 'b:', 'Baseline', 'LineWidth', 1);
xlabel('Encoder Output Position [deg]');
ylabel('Transmission Error [deg]');
title('Test 0 — TE vs Position (Hysteresis / Lissajous)');
legend('Location','best');

%% ---- Figure 6: TE distribution histogram ----
figure('Name','Fig 6 — TE Distribution', 'Position',[680 50 600 400]);
hold on; grid on;

histogram(trans_err, 50, 'FaceColor', [0.3 0.5 0.9], 'EdgeColor', 'none', ...
          'Normalization', 'probability', 'DisplayName', 'TE distribution');
xline(baseline_mean_te,      'b-',  sprintf('Mean = %+.4f deg', baseline_mean_te), 'LineWidth', 2);
xline(baseline_mean_te + global_std_te, 'b--', '+1 SD', 'LineWidth', 1);
xline(baseline_mean_te - global_std_te, 'b--', '-1 SD', 'LineWidth', 1);
xline( TIER1_TE_DEG + baseline_mean_te, 'y--', 'Tier 1', 'LineWidth', 1.2);
xline(-TIER1_TE_DEG + baseline_mean_te, 'y--',  '',      'LineWidth', 1.2);
xlabel('Transmission Error [deg]');
ylabel('Probability');
title(sprintf('Test 0 — TE Distribution (n=%d samples)', n_total));
legend('Location','best');

fprintf('All plots generated.\n\n');

%% ================================================================
%% FINAL SUMMARY — COPY TO LOGBOOK
%% ================================================================

fprintf('==========================================================\n');
fprintf('  LOGBOOK ENTRY — copy these values\n');
fprintf('==========================================================\n');
fprintf('  Date/time of analysis : %s\n', datestr(now));
fprintf('  Source file           : %s\n', CSV_FILENAME);
fprintf('  Gear ratio (metadata) : %.6f\n', gear_ratio_meta);
fprintf('  Gear ratio (data)     : %.6f  (%.4f%% discrepancy)\n', ...
        gr_from_data, 100*(gr_from_data - gear_ratio_meta)/gear_ratio_meta);
fprintf('  Encoder output range  : %.4f deg\n', output_range_deg);
fprintf('  -------------------------------------------------------\n');
fprintf('  BASELINE REFERENCE VALUES (use for all subsequent tests)\n');
fprintf('  Baseline mean TE      : %+.4f deg\n', baseline_mean_te);
fprintf('  Baseline std TE       : %.4f deg\n',  global_std_te);
fprintf('  Baseline RMS TE       : %.4f deg\n',  global_rms_te);
fprintf('  Baseline mean |curr|  : %.1f mA\n',   baseline_mean_current);
fprintf('  -------------------------------------------------------\n');
fprintf('  Tier 1 upper TE bound : %+.4f deg\n', baseline_mean_te + TIER1_TE_DEG);
fprintf('  Tier 1 lower TE bound : %+.4f deg\n', baseline_mean_te - TIER1_TE_DEG);
fprintf('  Tier 1 curr upper     : %.1f mA  (+5%% baseline)\n',  baseline_mean_current * 1.05);
fprintf('  Tier 2 curr upper     : %.1f mA  (+20%% baseline)\n', baseline_mean_current * 1.20);
fprintf('  -------------------------------------------------------\n');
fprintf('  Hysteresis (CW-CCW)   : %+.4f deg\n', hysteresis);
fprintf('  TE trend across 20 cy : %+.5f deg/cycle (negligible if < %.4f)\n', ...
        slope_te_per_cycle, TIER1_TE_DEG/n_cycles);
fprintf('  Tier 0/1/2/3 samples  : %d / %d / %d / %d\n', n_tier0, n_tier1, n_tier2, n_tier3);
fprintf('==========================================================\n');

end  %% ---- END MAIN ----


%% ================================================================
%% HELPER: read_metadata
%% Reads GEAR_RATIO, INPUT_RANGE_DEG, OUTPUT_RANGE_DEG from the
%% bottom of the CSV file
%% ================================================================
function meta = read_metadata(filename)
    meta.GEAR_RATIO      = NaN;
    meta.INPUT_RANGE_DEG  = NaN;
    meta.OUTPUT_RANGE_DEG = NaN;

    fid = fopen(filename, 'r');
    if fid < 0
        error('Cannot open file: %s', filename);
    end

    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line)
            lines{end+1} = line; %#ok<AGROW>
        end
    end
    fclose(fid);

    for i = 1:numel(lines)
        ln = strtrim(lines{i});

        if startsWith(ln, 'GEAR_RATIO,')
            parts = strsplit(ln, ',');
            meta.GEAR_RATIO = str2double(parts{2});
        elseif startsWith(ln, 'INPUT_RANGE_DEG,')
            parts = strsplit(ln, ',');
            meta.INPUT_RANGE_DEG = str2double(parts{2});
        elseif startsWith(ln, 'OUTPUT_RANGE_DEG,')
            parts = strsplit(ln, ',');
            meta.OUTPUT_RANGE_DEG = str2double(parts{2});
        end
    end

    if isnan(meta.GEAR_RATIO)
        warning('GEAR_RATIO not found in metadata — using NaN. Check CSV format.');
    end
end