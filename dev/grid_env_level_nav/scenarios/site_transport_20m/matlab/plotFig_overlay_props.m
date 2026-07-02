function plotFig_overlay_props(ax, props, labelProps)
%PLOTFIG_OVERLAY_PROPS Draw prop markers from layout registry.

if isempty(props)
    return;
end
for i = 1:numel(props)
    prop = props(i);
    xy = double(prop.local_xy_cm(:))';
    rgb = double(prop.mask_color_rgb(:))' / 255.0;
    plot(ax, xy(1), xy(2), 's', 'MarkerSize', 5, 'MarkerFaceColor', rgb, ...
        'MarkerEdgeColor', rgb * 0.6);
    if labelProps && isfield(prop, 'prop_type_id')
        text(ax, xy(1) + 4, xy(2) + 4, char(prop.prop_type_id), ...
            'FontSize', 6, 'Color', 'k', 'BackgroundColor', [1, 1, 1, 0.65]);
    end
end
end
