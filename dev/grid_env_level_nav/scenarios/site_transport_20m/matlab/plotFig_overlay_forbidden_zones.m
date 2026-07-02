function plotFig_overlay_forbidden_zones(ax, zones)
%PLOTFIG_OVERLAY_FORBIDDEN_ZONES Draw dashed rectangles for L1 zones.

if isempty(zones) || ~isstruct(zones)
    return;
end
for i = 1:numel(zones)
    rect = double(zones(i).rect_local_cm(:))';
    x0 = min(rect(1), rect(3));
    x1 = max(rect(1), rect(3));
    y0 = min(rect(2), rect(4));
    y1 = max(rect(2), rect(4));
    rectangle(ax, 'Position', [x0, y0, x1 - x0, y1 - y0], ...
        'EdgeColor', [0.79, 0.16, 0.16], 'LineStyle', '--', 'LineWidth', 1.5);
end
end
