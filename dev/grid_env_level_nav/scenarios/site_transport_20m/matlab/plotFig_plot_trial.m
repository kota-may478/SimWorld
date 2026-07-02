function entry = plotFig_plot_trial(trial, cfg)
%PLOTFIG_PLOT_TRIAL Load, render, and export one layout trial.

entry = struct( ...
    'layoutDir', trial.recordDir, ...
    'npzPath', trial.npzPath, ...
    'trajPath', trial.trajPath, ...
    'outputPath', '', ...
    'ok', false, ...
    'message', '');

if ~trial.ready
    entry.message = trial.skipReason;
    fprintf('  [skip] %s - %s\n', trial.name, entry.message);
    return;
end

try
    data = plotFig_load_costmap_data(trial.npzPath, trial.trajPath, cfg.registryPath);

    if strcmp(cfg.mode, 'single') && ~isempty(cfg.outputPath)
        outPath = cfg.outputPath;
    else
        outPath = plotFig_default_output_path(trial.npzPath);
    end

    hfig = plotFig_render_costmap(data, cfg);
    plotFig_export_figure(hfig, outPath, cfg.useFigTools);
    if ~cfg.visible
        close(hfig);
    end

    if ~isfile(outPath)
        error('plotFig:MissingOutput', 'expected output not found: %s', outPath);
    end

    entry.outputPath = outPath;
    entry.ok = true;
    entry.message = 'ok';
    fprintf('  [ok]   %s -> %s\n', trial.name, outPath);
catch ME
    entry.message = ME.message;
    fprintf('  [fail] %s - %s\n', trial.name, entry.message);
end
end
