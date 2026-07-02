function plotFig_overlay_trajectory(ax, trajectory)
%PLOTFIG_OVERLAY_TRAJECTORY Draw robot trajectory polyline.

if isempty(trajectory)
    return;
end
plot(ax, trajectory(:, 1), trajectory(:, 2), '-', 'Color', [0.0, 0.75, 1.0], 'LineWidth', 1.5);
end
