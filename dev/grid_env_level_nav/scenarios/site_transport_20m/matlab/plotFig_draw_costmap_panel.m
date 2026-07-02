function plotFig_draw_costmap_panel(ax, grid, data, extent, vmax, cmap, titleText, panelIndex)
%PLOTFIG_DRAW_COSTMAP_PANEL Draw one L0/L1/L2/merged panel.

axes(ax); %#ok<LAXES>
display = grid;
display(display >= data.lethal * 0.5) = data.lethal;
imagesc(ax, extent(1:2), extent(3:4), display');
set(ax, 'YDir', 'normal');
colormap(ax, cmap);
caxis(ax, [0, vmax]);
axis(ax, 'equal');
xlabel(ax, 'local X (cm)');
ylabel(ax, 'local Y (cm)');
title(ax, titleText);
colorbar(ax);

hold(ax, 'on');
if panelIndex == 2 || panelIndex == 4
    plotFig_overlay_forbidden_zones(ax, data.forbiddenZones);
end
if panelIndex == 3 || panelIndex == 4
    plotFig_overlay_props(ax, data.props, panelIndex == 3);
end
hold(ax, 'off');
end
