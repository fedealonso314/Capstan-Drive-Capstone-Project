function analyze_load_response_results()
% =========================================================================
%  LOAD RESPONSE TEST ANALYSIS — Test 1 (load_response_test_v2.m output)
%  Capstan Drive Reliability Project
%
%  Reads:  test1_load_response_*.csv  (produced by load_response_test_v2.m)
%  Computes:
%
%  1. Mean TE ± SD per load level (both passes, positive & negative leg)
%  2. Mean current ± SD per load level (both passes)
%  3. TE vs load calibration curve + linear/polynomial fit
%  4. Current vs load calibration curve + linear fit
%  5. Pass-to-pass repeatability (inter-pass variability)
%  6. Leg asymmetry: TE at +cmd vs -cmd endpoint per load
%  7. Load-stiffness estimate: TE slope (deg per % rated torque)
%  8. Current efficiency: mA per unit torque fraction
%  9. Failure tier assessment per load level
% 10. Comparison with baseline (Test 0) at 0% load
% 11. Comprehensive plots (5 figures)
% 12. Logbook-ready calibration table
%
%  HOW TO USE:
%    1. Place this file in the same folder as your test1_load_response_*.csv
%    2. Run: analyze_load_response_results()
%    3. Script auto-selects most recent CSV, or set CSV_FILENAME manually
% =========================================================================

clc;
clear;
close all;

%% ================================================================
%% TUNING
%% ================================================================

CSV_FILENAME = '';   % Leave '' to auto-select

% Failure tier TE thresholds (methodology doc §2)
TIER1_TE_DEG = 1.0;
TIER2_TE_DEG = 3.0;
TIER3_TE_DEG = 15.0;

% Baseline values from Test 0 (update after running analyze_baseline_results)
BASELINE_MEAN_TE      = -0.3407;   % deg
BASELINE_MEAN_CURRENT = 161.8;     % mA — NOTE: baseline was taken under load;
                                   % use no-load current here if available
% Polynomial degree for TE vs Load calibration fit (1=linear, 2=quadratic)
FIT_DEGREE = 2;

%% ================================================================
%% 1. LOAD DATA
%% ================================================================

if isempty(CSV_FILENAME)
    files = dir('test1_load_response_*.csv');
    if isempty(files)
        files = dir('/mnt/user-data/uploads/test1_load_response*.csv');
    end
    if isempty(files)
        error('No test1_load_response_*.csv found. Set CSV_FILENAME manually.');
    end
    [~, idx] = max([files.datenum]);
    CSV_FILENAME = fullfile(files(idx).folder, files(idx).name);
    fprintf('Auto-selected: %s\n\n', CSV_FILENAME);
end

metadata = read_metadata(CSV_FILENAME);
gear_ratio   = metadata.GEAR_RATIO;
input_range  = metadata.INPUT_RANGE_DEG;
output_range = metadata.OUTPUT_RANGE_DEG;

fprintf('Metadata:\n');
fprintf('  Gear ratio       : %.6f\n', gear_ratio);
fprintf('  Input range      : %.4f deg (Dynamixel)\n', input_range);
fprintf('  Output range     : %.4f deg (Encoder)\n\n', output_range);

%% Read data rows (stop before METADATA)
raw = readtable(CSV_FILENAME, 'VariableNamingRule', 'preserve');

% Drop trailing empty columns (artefact of extra commas in CSV)
non_empty_cols = ~all(ismissing(raw), 1);
raw = raw(:, non_empty_cols);

% Keep only numeric rows with valid PASS
valid = ~isnan(raw.PASS) & ~isnan(raw.LOAD_PCT);
raw   = raw(valid, :);

pass_col    = raw.PASS;
load_pct    = raw.LOAD_PCT;
cycle_col   = raw.CYCLE;
leg_col     = raw.LEG;
cmd_deg     = raw.CMD_OUTPUT_DEG;   % Commanded output position (±35 deg)
dxl_pos     = raw.DXL_POS_DEG;
enc_pos     = raw.ENC_POS_DEG;
trans_err   = raw.TRANS_ERR_DEG;
current_mA  = raw.CURRENT_mA;

passes     = unique(pass_col)';
load_lvls  = unique(load_pct)';
n_passes   = numel(passes);
n_loads    = numel(load_lvls);
n_total    = height(raw);

fprintf('Data loaded: %d samples | %d passes | %d load levels\n', n_total, n_passes, n_loads);
fprintf('Load levels: %s %%\n\n', num2str(load_lvls));

%% ================================================================
%% 2. PER LOAD LEVEL × PASS — STATISTICS
%% ================================================================

% Arrays: [n_loads × n_passes]
mean_te_mat   = NaN(n_loads, n_passes);
std_te_mat    = NaN(n_loads, n_passes);
mean_curr_mat = NaN(n_loads, n_passes);
std_curr_mat  = NaN(n_loads, n_passes);
n_samples_mat = zeros(n_loads, n_passes);

% Leg-separated [n_loads × n_passes × 2]
mean_te_leg   = NaN(n_loads, n_passes, 2);
std_te_leg    = NaN(n_loads, n_passes, 2);

for li = 1:n_loads
    for pi = 1:n_passes
        m = load_pct == load_lvls(li) & pass_col == passes(pi);
        if ~any(m), continue; end
        te_m  = trans_err(m);
        cur_m = abs(current_mA(m));

        mean_te_mat(li, pi)   = mean(te_m);
        std_te_mat(li, pi)    = std(te_m);
        mean_curr_mat(li, pi) = mean(cur_m);
        std_curr_mat(li, pi)  = std(cur_m);
        n_samples_mat(li, pi) = sum(m);

        for lg = 1:2
            ml = m & leg_col == lg;
            if any(ml)
                mean_te_leg(li, pi, lg) = mean(trans_err(ml));
                std_te_leg(li, pi, lg)  = std(trans_err(ml));
            end
        end
    end
end

%% ================================================================
%% 3. POOLED CALIBRATION VALUES (average across passes)
%% ================================================================

mean_te_pooled   = mean(mean_te_mat,   2, 'omitnan');   % n_loads × 1
std_te_pooled    = mean(std_te_mat,    2, 'omitnan');
mean_curr_pooled = mean(mean_curr_mat, 2, 'omitnan');
std_curr_pooled  = mean(std_curr_mat,  2, 'omitnan');

% Inter-pass range (repeatability metric)
te_pass_range   = max(mean_te_mat, [], 2) - min(mean_te_mat, [], 2);  % n_loads × 1
curr_pass_range = max(mean_curr_mat, [], 2) - min(mean_curr_mat, [], 2);

%% ================================================================
%% 4. CALIBRATION CURVE FITS
%% ================================================================

%% TE vs Load — polynomial fit
valid_fit = ~isnan(mean_te_pooled);
p_te = polyfit(load_lvls(valid_fit)', mean_te_pooled(valid_fit), FIT_DEGREE);
te_fit_vals = polyval(p_te, load_lvls);
te_fit_rms  = sqrt(mean((mean_te_pooled(valid_fit) - te_fit_vals(valid_fit)').^2));

% Also linear for comparison
p_te_lin = polyfit(load_lvls(valid_fit)', mean_te_pooled(valid_fit), 1);
te_lin_vals = polyval(p_te_lin, load_lvls);
te_lin_rms  = sqrt(mean((mean_te_pooled(valid_fit) - te_lin_vals(valid_fit)').^2));

%% Current vs Load — linear fit
valid_curr = ~isnan(mean_curr_pooled);
p_curr = polyfit(load_lvls(valid_curr)', mean_curr_pooled(valid_curr), 1);
curr_fit_vals = polyval(p_curr, load_lvls);
curr_fit_rms  = sqrt(mean((mean_curr_pooled(valid_curr) - curr_fit_vals(valid_curr)').^2));

%% TE slope (sensitivity): deg per % rated torque
te_sensitivity_deg_per_pct = p_te_lin(1);   % From linear fit
% More accurate: incremental slope between consecutive levels
te_increments = diff(mean_te_pooled) ./ diff(load_lvls');

%% Current sensitivity: mA per % rated torque
curr_sensitivity = p_curr(1);

%% Leg asymmetry at each load level (positive endpoint - negative endpoint)
leg_asym_te = mean_te_leg(:, :, 1) - mean_te_leg(:, :, 2);   % [n_loads × n_passes]
leg_asym_pooled = mean(leg_asym_te, 2, 'omitnan');

%% ================================================================
%% 5. PRINT RESULTS
%% ================================================================

fprintf('==========================================================\n');
fprintf('  TE vs LOAD CALIBRATION TABLE (pooled across passes)\n');
fprintf('  %-8s | %-12s | %-12s | %-12s | %-14s | %-10s | %-8s\n', ...
        'Load %', 'Mean TE', 'Std TE', 'Leg asym', 'Pass range', 'Mean |I|', 'Tier');
fprintf('  %s\n', repmat('-', 1, 84));
tier_str = {'T0','T1','T2','T3'};
for li = 1:n_loads
    te_abs = abs(mean_te_pooled(li));
    if te_abs >= TIER2_TE_DEG,      t_level = 3;
    elseif te_abs >= TIER1_TE_DEG,  t_level = 2;
    elseif te_abs >= abs(BASELINE_MEAN_TE) + 1.0, t_level = 1;
    else,                            t_level = 0;
    end
    fprintf('  %-8d | %+11.4f | %11.4f | %+11.4f | %13.4f | %9.1f | %-8s\n', ...
            load_lvls(li), mean_te_pooled(li), std_te_pooled(li), ...
            leg_asym_pooled(li), te_pass_range(li), mean_curr_pooled(li), ...
            tier_str{t_level + 1});
end
fprintf('==========================================================\n\n');

fprintf('TE CALIBRATION FIT:\n');
if FIT_DEGREE == 2
    fprintf('  Quadratic: TE = %.6f * load^2 + %.6f * load + %.6f\n', p_te(1), p_te(2), p_te(3));
else
    fprintf('  Linear: TE = %.6f * load + %.6f\n', p_te(1), p_te(2));
end
fprintf('  Quadratic RMS residual: %.4f deg\n', te_fit_rms);
fprintf('  Linear:    TE = %.6f * load + %.6f\n', p_te_lin(1), p_te_lin(2));
fprintf('  Linear RMS residual:    %.4f deg\n', te_lin_rms);
fprintf('  TE sensitivity (linear): %.5f deg / %% rated torque\n\n', te_sensitivity_deg_per_pct);

fprintf('INCREMENTAL TE SENSITIVITY:\n');
for li = 1:n_loads-1
    fprintf('  %d%% to %d%% : %+.5f deg / %% torque\n', ...
            load_lvls(li), load_lvls(li+1), te_increments(li));
end
fprintf('\n');

fprintf('CURRENT CALIBRATION FIT:\n');
fprintf('  Linear: |I| = %.4f * load + %.4f  (mA)\n', p_curr(1), p_curr(2));
fprintf('  RMS residual: %.2f mA\n', curr_fit_rms);
fprintf('  Current sensitivity: %.3f mA / %% rated torque\n\n', curr_sensitivity);

fprintf('LEG ASYMMETRY (TE at +cmd endpoint minus -cmd endpoint):\n');
for li = 1:n_loads
    fprintf('  Load %d%%: asymmetry = %+.4f deg\n', load_lvls(li), leg_asym_pooled(li));
end
fprintf('\n');

fprintf('PASS-TO-PASS REPEATABILITY:\n');
fprintf('  Max TE range across passes: %.4f deg (at %d%% load)\n', ...
        max(te_pass_range), load_lvls(te_pass_range == max(te_pass_range)));
fprintf('  Max I range across passes : %.2f mA (at %d%% load)\n\n', ...
        max(curr_pass_range), load_lvls(curr_pass_range == max(curr_pass_range)));

fprintf('COMPARISON WITH TEST 0 BASELINE (at 0%% load):\n');
if any(load_lvls == 0)
    te_0load = mean_te_pooled(load_lvls == 0);
    fprintf('  Test 1 TE at 0%% load   : %+.4f deg\n', te_0load);
    fprintf('  Test 0 baseline TE     : %+.4f deg\n', BASELINE_MEAN_TE);
    fprintf('  Difference             : %+.4f deg\n\n', te_0load - BASELINE_MEAN_TE);
end

%% ================================================================
%% 6. SAVE CALIBRATION CSV
%% ================================================================

T_cal = table(load_lvls', mean_te_pooled, std_te_pooled, mean_curr_pooled, std_curr_pooled, ...
              te_pass_range, leg_asym_pooled, te_fit_vals', curr_fit_vals', ...
    'VariableNames', {'LOAD_PCT','MEAN_TE_DEG','STD_TE_DEG','MEAN_CURR_mA','STD_CURR_mA', ...
                      'TE_PASS_RANGE_DEG','LEG_ASYM_DEG','TE_FIT_DEG','CURR_FIT_mA'});

out_filename = sprintf('test1_analysis_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
writetable(T_cal, out_filename);

fid = fopen(out_filename, 'a');
fprintf(fid, '\nCALIBRATION_FITS\n');
if FIT_DEGREE == 2
    fprintf(fid, 'TE_POLY_COEFFS_A2_A1_A0,%.8f,%.8f,%.8f\n', p_te(1), p_te(2), p_te(3));
else
    fprintf(fid, 'TE_POLY_COEFFS_A1_A0,%.8f,%.8f\n', p_te(1), p_te(2));
end
fprintf(fid, 'TE_LINEAR_SLOPE,%.8f\n',    p_te_lin(1));
fprintf(fid, 'TE_LINEAR_INTERCEPT,%.8f\n',p_te_lin(2));
fprintf(fid, 'TE_LIN_RMS_DEG,%.6f\n',     te_lin_rms);
fprintf(fid, 'CURR_LINEAR_SLOPE,%.6f\n',  p_curr(1));
fprintf(fid, 'CURR_LINEAR_INTERCEPT,%.6f\n', p_curr(2));
fprintf(fid, 'CURR_FIT_RMS_mA,%.4f\n',    curr_fit_rms);
fprintf(fid, '\nMETADATA_ECHO\n');
fprintf(fid, 'GEAR_RATIO,%.6f\n',         gear_ratio);
fprintf(fid, 'INPUT_RANGE_DEG,%.4f\n',    input_range);
fprintf(fid, 'OUTPUT_RANGE_DEG,%.4f\n',   output_range);
fprintf(fid, 'BASELINE_MEAN_TE_DEG_REF,%.4f\n', BASELINE_MEAN_TE);
fprintf(fid, 'BASELINE_MEAN_CURR_REF,%.2f\n',   BASELINE_MEAN_CURRENT);
fclose(fid);

fprintf('Analysis saved to: %s\n\n', out_filename);

%% ================================================================
%% 7. PLOTS
%% ================================================================

pass_colors = [0.1 0.4 0.8; 0.8 0.2 0.1; 0.1 0.7 0.3];
leg_markers = {'o', 's'};
leg_names   = {'+cmd endpoint', '-cmd endpoint'};

%% ---- Figure 1: Mean TE ± SD vs Load — both passes overlaid ----
figure('Name','Fig 1 — TE vs Load', 'Position',[50 600 900 400]);
hold on; grid on;
for pi = 1:n_passes
    te_p  = mean_te_mat(:, pi);
    sd_p  = std_te_mat(:, pi);
    valid = ~isnan(te_p);
    errorbar(load_lvls(valid), te_p(valid), sd_p(valid), ...
             ['-' leg_markers{min(pi,2)}], 'Color', pass_colors(pi,:), ...
             'LineWidth', 1.5, 'MarkerSize', 6, ...
             'DisplayName', sprintf('Pass %d (mean ± 1 SD)', passes(pi)));
end
% Calibration curve
x_fit = linspace(0, max(load_lvls), 200);
if FIT_DEGREE == 2
    plot(x_fit, polyval(p_te, x_fit), 'k-', 'LineWidth', 2.0, ...
         'DisplayName', sprintf('Quadratic fit (RMS=%.3f deg)', te_fit_rms));
end
plot(x_fit, polyval(p_te_lin, x_fit), 'k--', 'LineWidth', 1.2, ...
     'DisplayName', sprintf('Linear fit (%.4f deg/%%)', te_sensitivity_deg_per_pct));
yline(BASELINE_MEAN_TE,         'b:',  'Test 0 baseline', 'LineWidth', 1.5);
yline(BASELINE_MEAN_TE + 1.0,   'y--', 'Baseline ±1 deg', 'LineWidth', 1.0);
yline(BASELINE_MEAN_TE - 1.0,   'y--', '',                'LineWidth', 1.0, 'HandleVisibility','off');
yline(-TIER2_TE_DEG, 'r--', 'Tier 2 (-3 deg)', 'LineWidth', 1.0);
xlabel('Load [% of rated torque]');
ylabel('Mean Transmission Error [deg]');
title('Test 1 — TE vs Load Level (Calibration Curve)');
legend('Location','southwest');

%% ---- Figure 2: Mean Current vs Load — both passes ----
figure('Name','Fig 2 — Current vs Load', 'Position',[50 160 900 400]);
hold on; grid on;
for pi = 1:n_passes
    cur_p = mean_curr_mat(:, pi);
    sd_p  = std_curr_mat(:, pi);
    valid = ~isnan(cur_p);
    errorbar(load_lvls(valid), cur_p(valid), sd_p(valid), ...
             ['-' leg_markers{min(pi,2)}], 'Color', pass_colors(pi,:), ...
             'LineWidth', 1.5, 'MarkerSize', 6, ...
             'DisplayName', sprintf('Pass %d', passes(pi)));
end
plot(x_fit, polyval(p_curr, x_fit), 'k-', 'LineWidth', 2.0, ...
     'DisplayName', sprintf('Linear fit (%.2f mA/%%)', curr_sensitivity));
yline(BASELINE_MEAN_CURRENT,         'b:',  'Test 0 |I| baseline', 'LineWidth', 1.5);
yline(BASELINE_MEAN_CURRENT * 1.20,  'r--', '+20% Tier 2',         'LineWidth', 1.0);
yline(BASELINE_MEAN_CURRENT * 1.05,  'y--', '+5% Tier 1',          'LineWidth', 1.0);
xlabel('Load [% of rated torque]');
ylabel('Mean |Current| [mA]');
title('Test 1 — Motor Current vs Load Level');
legend('Location','northwest');

%% ---- Figure 3: TE at positive vs negative leg — all loads ----
figure('Name','Fig 3 — TE by Leg and Load', 'Position',[1000 600 900 400]);
hold on; grid on;
leg_colors = [0.1 0.4 0.8; 0.8 0.2 0.1];
for lg = 1:2
    te_lg_pooled = mean(mean_te_leg(:, :, lg), 2, 'omitnan');
    sd_lg_pooled = mean(std_te_leg(:, :, lg),  2, 'omitnan');
    valid = ~isnan(te_lg_pooled);
    errorbar(load_lvls(valid) + (lg-1.5)*1.5, te_lg_pooled(valid), sd_lg_pooled(valid), ...
             ['-' leg_markers{lg}], 'Color', leg_colors(lg,:), ...
             'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', leg_names{lg});
end
yline(BASELINE_MEAN_TE, 'b:', 'Baseline', 'LineWidth', 1.5);
xlabel('Load [% of rated torque]');
ylabel('Mean Transmission Error [deg]');
title('Test 1 — TE at Each Endpoint vs Load (Leg Asymmetry)');
legend('Location','southwest');

%% ---- Figure 4: Pass-to-pass repeatability (TE range per load level) ----
figure('Name','Fig 4 — Pass Repeatability', 'Position',[1000 160 600 380]);
hold on; grid on;
bar(load_lvls, te_pass_range, 'FaceColor', [0.3 0.5 0.9], 'EdgeColor', 'none');
yline(0.1, 'k--', 'Reference 0.1 deg', 'LineWidth', 1);
xlabel('Load [% rated torque]');
ylabel('TE Range Across Passes [deg]');
title('Test 1 — Pass-to-Pass TE Repeatability');

%% ---- Figure 5: Raw TE scatter — all samples coloured by load level ----
figure('Name','Fig 5 — TE Scatter by Load', 'Position',[520 380 900 400]);
hold on; grid on;
load_cmap = parula(n_loads);
for li = 1:n_loads
    m = load_pct == load_lvls(li);
    scatter(repmat(load_lvls(li), sum(m), 1) + randn(sum(m),1)*0.5, ...
            trans_err(m), 8, load_cmap(li,:), 'filled', 'MarkerFaceAlpha', 0.4, ...
            'DisplayName', sprintf('%d%%', load_lvls(li)));
end
% Overlay pooled means
plot(load_lvls, mean_te_pooled, 'k-o', 'LineWidth', 2.5, 'MarkerSize', 8, ...
     'MarkerFaceColor','k', 'DisplayName', 'Pooled mean');
yline(BASELINE_MEAN_TE, 'b:', 'Baseline', 'LineWidth', 1.5);
yline(-TIER2_TE_DEG, 'r--', 'Tier 2', 'LineWidth', 1);
xlabel('Load [% rated torque]');
ylabel('Transmission Error [deg]');
title('Test 1 — TE Sample Scatter by Load Level');
legend('Location','southwest', 'NumColumns', 2);

fprintf('All plots generated.\n\n');

%% ================================================================
%% LOGBOOK SUMMARY
%% ================================================================

fprintf('==========================================================\n');
fprintf('  LOGBOOK ENTRY — Test 1 Load Response\n');
fprintf('==========================================================\n');
fprintf('  Date/time         : %s\n', datestr(now));
fprintf('  Source file       : %s\n', CSV_FILENAME);
fprintf('  Gear ratio        : %.6f\n', gear_ratio);
fprintf('  Output range      : %.4f deg\n', output_range);
fprintf('  Passes completed  : %d\n', n_passes);
fprintf('  -------------------------------------------------------\n');
fprintf('  CALIBRATION CURVES (use for endurance test interpretation)\n');
if FIT_DEGREE == 2
    fprintf('  TE(load)  = %.6f*load^2 + %.6f*load + %.6f  [deg]\n', p_te(1), p_te(2), p_te(3));
else
    fprintf('  TE(load)  = %.6f*load + %.6f  [deg]\n', p_te(1), p_te(2));
end
fprintf('  I(load)   = %.4f*load + %.4f  [mA]\n', p_curr(1), p_curr(2));
fprintf('  TE sensitivity    : %.5f deg / %% rated torque\n', te_sensitivity_deg_per_pct);
fprintf('  I  sensitivity    : %.3f mA / %% rated torque\n', curr_sensitivity);
fprintf('  -------------------------------------------------------\n');
fprintf('  PER-LEVEL SUMMARY\n');
fprintf('  %-8s  %-12s  %-12s  %-12s\n', 'Load %', 'Mean TE [deg]', 'Std TE [deg]', 'Mean |I| [mA]');
for li = 1:n_loads
    fprintf('  %-8d  %+11.4f  %11.4f  %12.1f\n', ...
            load_lvls(li), mean_te_pooled(li), std_te_pooled(li), mean_curr_pooled(li));
end
fprintf('  -------------------------------------------------------\n');
fprintf('  BASELINE COMPARISON (Test 0 vs Test 1 at 0%% load)\n');
if any(load_lvls == 0)
    fprintf('  Test 0 baseline TE  : %+.4f deg\n', BASELINE_MEAN_TE);
    fprintf('  Test 1 TE at 0%%    : %+.4f deg\n', mean_te_pooled(load_lvls == 0));
    fprintf('  Offset              : %+.4f deg\n', mean_te_pooled(load_lvls == 0) - BASELINE_MEAN_TE);
end
fprintf('==========================================================\n');

end  %% ---- END MAIN ----


%% ================================================================
%% HELPER: read_metadata
%% ================================================================
function meta = read_metadata(filename)
    meta.GEAR_RATIO       = NaN;
    meta.INPUT_RANGE_DEG  = NaN;
    meta.OUTPUT_RANGE_DEG = NaN;

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
end