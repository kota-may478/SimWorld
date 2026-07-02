function batchDir = plotFig_resolve_batch_dir(cfg)
%PLOTFIG_RESOLVE_BATCH_DIR Pick batch directory from cfg or newest out/layout_batch_*.

if isfield(cfg, 'batchDir') && ~isempty(cfg.batchDir)
    batchDir = char(cfg.batchDir);
    if ~isfolder(batchDir)
        error('plotFig:BadDir', 'Batch directory not found: %s', batchDir);
    end
    fprintf('Using batch directory: %s\n', batchDir);
    return;
end

scriptDir = fileparts(mfilename('fullpath'));
outDir = fullfile(scriptDir, '..', 'out');
if ~isfolder(outDir)
    error('plotFig:NoOutDir', 'Output directory not found: %s', outDir);
end

batches = dir(fullfile(outDir, 'layout_batch_*'));
batches = batches([batches.isdir]);
if isempty(batches)
    error('plotFig:NoBatch', 'No layout_batch_* folder under: %s', outDir);
end

names = {batches.name};
[~, order] = sort(names);
order = order(end:-1:1);
batchDir = fullfile(batches(order(1)).folder, batches(order(1)).name);
fprintf('Using batch directory: %s\n', batchDir);
end
