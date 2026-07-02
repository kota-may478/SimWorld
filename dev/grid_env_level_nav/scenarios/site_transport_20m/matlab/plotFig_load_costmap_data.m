function data = plotFig_load_costmap_data(npzPath, trajPath, registryPath)
%PLOTFIG_LOAD_COSTMAP_DATA Load npz layers, trajectory JSON, and optional registry.

if nargin < 3
    registryPath = '';
end

if ~isfile(npzPath)
    error('plotFig_load_costmap_data:MissingNpz', 'NPZ not found: %s', npzPath);
end
if ~isfile(trajPath)
    error('plotFig_load_costmap_data:MissingTraj', 'Trajectory JSON not found: %s', trajPath);
end

raw = load_site_transport_npz(npzPath);
required = {'l0', 'l1', 'l2', 'lethal_cost'};
for k = 1:numel(required)
    if ~isfield(raw, required{k})
        error('plotFig_load_costmap_data:BadNpz', 'NPZ missing field: %s', required{k});
    end
end

traj = jsondecode(fileread(trajPath));

data.lethal = double(raw.lethal_cost);
if ~isscalar(data.lethal) && numel(data.lethal) == 1
    data.lethal = data.lethal(1);
end
data.l0 = double(raw.l0);
data.l1 = double(raw.l1);
data.l2 = double(raw.l2);
if isfield(raw, 'merged')
    data.merged = double(raw.merged);
else
    data.merged = max(max(data.l0, data.l1), data.l2);
end

data.regionSize = 2000.0;
if isfield(traj, 'region_size_cm')
    data.regionSize = double(traj.region_size_cm);
elseif isfield(traj, 'metrics') && isfield(traj.metrics, 'region_size_cm')
    data.regionSize = double(traj.metrics.region_size_cm);
end

data.layoutId = read_layout_id(traj);
data.trajectory = read_xy_pairs(traj, 'trajectory_local_cm');
data.forbiddenZones = read_forbidden_zones(traj);
data.robotStart = read_xy(traj, 'robot_start_local_cm', [100, 100]);
data.humanoid = read_xy(traj, 'humanoid_local_cm', [100, 30]);
data.materialPickup = read_xy(traj, 'material_pickup_local_cm', [1850, 1850]);
data.props = [];

if isempty(registryPath)
    registryPath = default_registry_path(data.layoutId);
end
if ~isempty(registryPath) && isfile(registryPath)
    reg = jsondecode(fileread(registryPath));
    if isfield(reg, 'props')
        data.props = reg.props;
    end
    if isfield(reg, 'layout_id')
        data.layoutId = char(reg.layout_id);
    end
end
end

function layoutId = read_layout_id(traj)
layoutId = '';
if isfield(traj, 'metrics') && isfield(traj.metrics, 'layout_id')
    layoutId = char(traj.metrics.layout_id);
elseif isfield(traj, 'layout_id')
    layoutId = char(traj.layout_id);
end
end

function zones = read_forbidden_zones(traj)
zones = [];
if isfield(traj, 'forbidden_zones')
    zones = traj.forbidden_zones;
elseif isfield(traj, 'metrics') && isfield(traj.metrics, 'rules') ...
        && isfield(traj.metrics.rules, 'forbidden_zones')
    zones = traj.metrics.rules.forbidden_zones;
end
end

function xy = read_xy(traj, fieldName, defaultXY)
xy = defaultXY;
if isfield(traj, fieldName)
    xy = double(traj.(fieldName)(:))';
end
end

function pairs = read_xy_pairs(traj, fieldName)
pairs = zeros(0, 2);
if ~isfield(traj, fieldName)
    return;
end
raw = traj.(fieldName);
if isempty(raw)
    return;
end
if iscell(raw)
    n = numel(raw);
    pairs = zeros(n, 2);
    for i = 1:n
        pairs(i, :) = double(raw{i}(:))';
    end
else
    pairs = double(raw);
    if size(pairs, 2) ~= 2
        pairs = pairs';
    end
end
end

function registryPath = default_registry_path(layoutId)
registryPath = '';
if isempty(layoutId)
    return;
end
scriptDir = fileparts(mfilename('fullpath'));
registryPath = fullfile(scriptDir, '..', '..', '..', 'cache', 'registries', ...
    sprintf('site_transport_20m_%s.json', layoutId));
end
