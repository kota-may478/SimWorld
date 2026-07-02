function cmap = site_transport_costmap_colormap(n)
%SITE_TRANSPORT_COSTMAP_COLORMAP Green-yellow-red colormap (matplotlib RdYlGn_r-like).
if nargin < 1
    n = 256;
end
t = linspace(0, 1, n)';
r = min(1, max(0, 1.5 * t - 0.25));
g = min(1, max(0, 1.2 - abs(t - 0.5) * 2.4));
b = min(1, max(0, 1.0 - 1.8 * t));
cmap = [r, g, b];
end
