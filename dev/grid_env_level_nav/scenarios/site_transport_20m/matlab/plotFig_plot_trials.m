function summary = plotFig_plot_trials(trials, cfg)
%PLOTFIG_PLOT_TRIALS Plot all discovered trials sequentially.

summary = struct('layoutDir', {}, 'npzPath', {}, 'trajPath', {}, ...
    'outputPath', {}, 'ok', {}, 'message', {});

fprintf('Plotting %d trial(s)\n', numel(trials));
for i = 1:numel(trials)
    entry = plotFig_plot_trial(trials(i), cfg);
    summary(end + 1) = entry; %#ok<AGROW>
end

okCount = sum([summary.ok]);
fprintf('Done: %d/%d figures exported.\n', okCount, numel(summary));
end
