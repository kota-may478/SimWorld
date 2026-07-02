function trials = plotFig_discover_trials(batchDir)
%PLOTFIG_DISCOVER_TRIALS Find layout_*_test folders and artifact paths.

if ~isfolder(batchDir)
    error('plotFig:BadDir', 'Batch directory not found: %s', batchDir);
end

layoutDirs = dir(fullfile(batchDir, 'layout_*_test'));
layoutDirs = layoutDirs([layoutDirs.isdir]);
[~, order] = sort({layoutDirs.name});
layoutDirs = layoutDirs(order);

trials = repmat(struct( ...
    'name', '', ...
    'recordDir', '', ...
    'npzPath', '', ...
    'trajPath', '', ...
    'ready', false, ...
    'skipReason', ''), 0, 1);

for i = 1:numel(layoutDirs)
    recordDir = fullfile(batchDir, layoutDirs(i).name);
    trial = struct( ...
        'name', layoutDirs(i).name, ...
        'recordDir', recordDir, ...
        'npzPath', '', ...
        'trajPath', '', ...
        'ready', false, ...
        'skipReason', '');

    npzFiles = dir(fullfile(recordDir, 'site_transport_costmap_*.npz'));
    trajFiles = dir(fullfile(recordDir, 'site_transport_trajectory_*.json'));
    if isempty(npzFiles)
        trial.skipReason = 'missing site_transport_costmap_*.npz';
    elseif isempty(trajFiles)
        trial.skipReason = 'missing site_transport_trajectory_*.json';
    else
        trial.npzPath = fullfile(npzFiles(1).folder, npzFiles(1).name);
        trial.trajPath = fullfile(trajFiles(1).folder, trajFiles(1).name);
        trial.ready = true;
        trial.skipReason = '';
    end
    trials(end + 1) = trial; %#ok<AGROW>
end

fprintf('Discovered %d trial folder(s) under %s\n', numel(trials), batchDir);
end
