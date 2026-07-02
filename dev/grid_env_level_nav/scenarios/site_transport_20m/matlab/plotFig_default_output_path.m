function outPath = plotFig_default_output_path(npzPath)
%PLOTFIG_DEFAULT_OUTPUT_PATH Derive costMap_matlab_<suffix>.png beside npz.

[npzDir, npzName, ~] = fileparts(npzPath);
token = regexp(npzName, 'site_transport_costmap_(.+)$', 'tokens', 'once');
if isempty(token)
    suffix = 'costmap';
else
    suffix = token{1};
end
outPath = fullfile(npzDir, sprintf('costMap_matlab_%s.png', suffix));
end
