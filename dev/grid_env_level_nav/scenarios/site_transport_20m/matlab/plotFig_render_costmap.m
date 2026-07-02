function hfig = plotFig_render_costmap(data, cfg)
%PLOTFIG_RENDER_COSTMAP Build 4-panel costmap figure from loaded data.

if cfg.visible
    vis = 'on';
else
    vis = 'off';
end

hfig = figure('Color', 'w', 'Position', [80, 80, 1320, 420], 'Visible', vis);
tiled = tiledlayout(hfig, 1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

layerNames = {'L0 (NavMesh)', 'L1 (forbidden)', 'L2 (perception)', 'Merged + path'};
layerArrays = {data.l0, data.l1, data.l2, data.merged};
extent = [0, data.regionSize, 0, data.regionSize];
vmax = max(10.0, data.lethal);
cmap = site_transport_costmap_colormap();

for i = 1:3
    ax = nexttile(tiled, i);
    plotFig_draw_costmap_panel(ax, layerArrays{i}, data, extent, vmax, cmap, layerNames{i}, i);
end

ax = nexttile(tiled, 4);
plotFig_draw_costmap_panel(ax, data.merged, data, extent, vmax, cmap, layerNames{4}, 4);
plotFig_overlay_trajectory(ax, data.trajectory);
plotFig_overlay_markers(ax, data);

if nnz(data.l1) > 0
    mergedLabel = 'L0+L1+L2';
else
    mergedLabel = 'L0+L2';
end
title(tiled, sprintf('Site transport 20m x 20m - costmaps (%s, %s)', ...
    data.layoutId, mergedLabel), 'FontWeight', 'bold');

if cfg.useFigTools && exist('pubfig', 'file') == 2
    pf = pubfig(hfig);
    pf.Grid = 'off';
end
end
