function cfg = plotFig_parse_config(varargin)
%PLOTFIG_PARSE_CONFIG Build run configuration from plotFig arguments.
%
% Modes:
%   batch  - plotFig() or plotFig(batchDir, ...)
%   single - plotFig(npzPath, trajPath, ...)

cfg = struct( ...
    'mode', 'batch', ...
    'batchDir', '', ...
    'npzPath', '', ...
    'trajPath', '', ...
    'outputPath', '', ...
    'registryPath', '', ...
    'useFigTools', true, ...
    'visible', false);

if nargin == 0
    return;
end

if nargin == 1 && (ischar(varargin{1}) || isstring(varargin{1})) && isfolder(varargin{1})
    cfg.batchDir = char(varargin{1});
    cfg = apply_name_value_pairs(cfg, varargin{2:end});
    return;
end

if nargin >= 2 && (ischar(varargin{1}) || isstring(varargin{1})) ...
        && (ischar(varargin{2}) || isstring(varargin{2}))
    cfg.mode = 'single';
    cfg.npzPath = char(varargin{1});
    cfg.trajPath = char(varargin{2});
    cfg.visible = true;
    cfg = apply_name_value_pairs(cfg, varargin{3:end});
    return;
end

error('plotFig:BadArgs', ...
    ['Usage: plotFig | plotFig(batchDir) | plotFig(npzPath, trajPath, ...)\n', ...
     'Options: UseFigTools, Visible, OutputPath, RegistryPath']);
end

function cfg = apply_name_value_pairs(cfg, args)
if isempty(args)
    return;
end
if mod(numel(args), 2) ~= 0
    error('plotFig:BadArgs', 'Name-value arguments must come in pairs.');
end
for i = 1:2:numel(args)
    name = lower(strrep(char(args{i}), '_', ''));
    value = args{i + 1};
    switch name
        case 'usefigtools'
            cfg.useFigTools = logical(value);
        case 'visible'
            cfg.visible = logical(value);
        case 'outputpath'
            cfg.outputPath = char(value);
        case 'registrypath'
            cfg.registryPath = char(value);
        case 'batchdir'
            cfg.batchDir = char(value);
        otherwise
            error('plotFig:BadOption', 'Unknown option: %s', char(args{i}));
    end
end
end
