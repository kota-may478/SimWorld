function plotFig_overlay_markers(ax, data)
%PLOTFIG_OVERLAY_MARKERS Draw start, crate, and humanoid markers.

plot(ax, data.robotStart(1), data.robotStart(2), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', [0, 0.5, 0.5]);
plot(ax, data.materialPickup(1), data.materialPickup(2), 's', ...
    'MarkerSize', 9, 'MarkerFaceColor', [1, 0.84, 0], 'MarkerEdgeColor', [0.7, 0.55, 0]);
plot(ax, data.humanoid(1), data.humanoid(2), '^', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', [0.6, 0, 0.6]);
end
