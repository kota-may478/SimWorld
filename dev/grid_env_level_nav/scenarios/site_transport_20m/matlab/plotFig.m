% plotFig.m — site_transport_20m costmap plotting (manual script)
%
% 上から %% セクション順に実行してください。
% データ読み込みは plotFig_* ヘルパーを使用し、プロットはこのファイル内に直書きしています。
% 別レイアウトを描くときは Step 3 の trialIdx を 1..10 に変えて Step 3 以降を再実行してください。

%% Step 0: ヘルパー関数のパスを通す
plotFig_ensure_path();

%% Step 1: オプション（必要なら編集）
useFigTools = true;
showFigure = true;
registryPath = '';  % 空なら layout_id から registry JSON を自動解決
% batchDir = fullfile(pwd, 'out', 'layout_batch_20260629T091634Z');  % 上書きする場合

%% Step 2: バッチフォルダと試行一覧を取得
cfg = plotFig_parse_config();
if exist('batchDir', 'var') && ~isempty(batchDir)
    cfg.batchDir = batchDir;
end
batchDir = plotFig_resolve_batch_dir(cfg);
trials = plotFig_discover_trials(batchDir);

%% Step 3: 試行を選んでデータ読み込み（layout_01 → trialIdx=1）
trialIdx = 1;
trial = trials(trialIdx);
data = plotFig_load_costmap_data(trial.npzPath, trial.trajPath, registryPath);
outPath = plotFig_default_output_path(trial.npzPath);

extent = [0, data.regionSize, 0, data.regionSize];
vmax = max(10.0, data.lethal);
cmap = site_transport_costmap_colormap();

fprintf('Plotting %s (%s)\n', trial.name, data.layoutId);

%% Step 4: figure 作成
if showFigure
    vis = 'on';
else
    vis = 'off';
end
hfig = figure('Color', 'w', 'Position', [80, 80, 1320, 420], 'Visible', vis);
tiled = tiledlayout(hfig, 1, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

%% Step 5: Panel 1 — L0 (NavMesh)
ax1 = nexttile(tiled, 1);
displayL0 = data.l0;
displayL0(displayL0 >= data.lethal * 0.5) = data.lethal;
imagesc(ax1, extent(1:2), extent(3:4), displayL0');
set(ax1, 'YDir', 'normal');
colormap(ax1, cmap);
caxis(ax1, [0, vmax]);
axis(ax1, 'equal');
xlabel(ax1, 'local X (cm)');
ylabel(ax1, 'local Y (cm)');
title(ax1, 'L0 (NavMesh)');
colorbar(ax1);

%% Step 6: Panel 2 — L1 (forbidden)
ax2 = nexttile(tiled, 2);
displayL1 = data.l1;
displayL1(displayL1 >= data.lethal * 0.5) = data.lethal;
imagesc(ax2, extent(1:2), extent(3:4), displayL1');
set(ax2, 'YDir', 'normal');
colormap(ax2, cmap);
caxis(ax2, [0, vmax]);
axis(ax2, 'equal');
xlabel(ax2, 'local X (cm)');
ylabel(ax2, 'local Y (cm)');
title(ax2, 'L1 (forbidden)');
colorbar(ax2);
hold(ax2, 'on');
zones = data.forbiddenZones;
if ~isempty(zones) && isstruct(zones)
    for zi = 1:numel(zones)
        rect = double(zones(zi).rect_local_cm(:))';
        x0 = min(rect(1), rect(3));
        x1 = max(rect(1), rect(3));
        y0 = min(rect(2), rect(4));
        y1 = max(rect(2), rect(4));
        rectangle(ax2, 'Position', [x0, y0, x1 - x0, y1 - y0], ...
            'EdgeColor', [0.79, 0.16, 0.16], 'LineStyle', '--', 'LineWidth', 1.5);
    end
end
hold(ax2, 'off');

%% Step 7: Panel 3 — L2 (perception)
ax3 = nexttile(tiled, 3);
displayL2 = data.l2;
displayL2(displayL2 >= data.lethal * 0.5) = data.lethal;
imagesc(ax3, extent(1:2), extent(3:4), displayL2');
set(ax3, 'YDir', 'normal');
colormap(ax3, cmap);
caxis(ax3, [0, vmax]);
axis(ax3, 'equal');
xlabel(ax3, 'local X (cm)');
ylabel(ax3, 'local Y (cm)');
title(ax3, 'L2 (perception)');
colorbar(ax3);
hold(ax3, 'on');
props = data.props;
if ~isempty(props)
    for pi = 1:numel(props)
        xy = double(props(pi).local_xy_cm(:))';
        rgb = double(props(pi).mask_color_rgb(:))' / 255.0;
        plot(ax3, xy(1), xy(2), 's', 'MarkerSize', 5, ...
            'MarkerFaceColor', rgb, 'MarkerEdgeColor', rgb * 0.6);
        if isfield(props(pi), 'prop_type_id')
            text(ax3, xy(1) + 4, xy(2) + 4, char(props(pi).prop_type_id), ...
                'FontSize', 6, 'Color', 'k', 'Interpreter', 'none', ...
                'BackgroundColor', [1, 1, 1, 0.65]);
        end
    end
end
hold(ax3, 'off');

%% Step 8: Panel 4 — Merged + path
ax4 = nexttile(tiled, 4);
displayMerged = data.merged;
displayMerged(displayMerged >= data.lethal * 0.5) = data.lethal;
imagesc(ax4, extent(1:2), extent(3:4), displayMerged');
set(ax4, 'YDir', 'normal');
colormap(ax4, cmap);
caxis(ax4, [0, vmax]);
axis(ax4, 'equal');
xlabel(ax4, 'local X (cm)');
ylabel(ax4, 'local Y (cm)');
title(ax4, 'Merged + path');
colorbar(ax4);
hold(ax4, 'on');
if ~isempty(zones) && isstruct(zones)
    for zi = 1:numel(zones)
        rect = double(zones(zi).rect_local_cm(:))';
        x0 = min(rect(1), rect(3));
        x1 = max(rect(1), rect(3));
        y0 = min(rect(2), rect(4));
        y1 = max(rect(2), rect(4));
        rectangle(ax4, 'Position', [x0, y0, x1 - x0, y1 - y0], ...
            'EdgeColor', [0.79, 0.16, 0.16], 'LineStyle', '--', 'LineWidth', 1.5);
    end
end
if ~isempty(props)
    for pi = 1:numel(props)
        xy = double(props(pi).local_xy_cm(:))';
        rgb = double(props(pi).mask_color_rgb(:))' / 255.0;
        plot(ax4, xy(1), xy(2), 's', 'MarkerSize', 5, ...
            'MarkerFaceColor', rgb, 'MarkerEdgeColor', rgb * 0.6);
    end
end
traj = data.trajectory;
if ~isempty(traj)
    plot(ax4, traj(:, 1), traj(:, 2), '-', 'Color', [0.0, 0.75, 1.0], 'LineWidth', 1.5);
end
plot(ax4, data.robotStart(1), data.robotStart(2), 'o', ...
    'MarkerSize', 7, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', [0, 0.5, 0.5]);
plot(ax4, data.materialPickup(1), data.materialPickup(2), 's', ...
    'MarkerSize', 9, 'MarkerFaceColor', [1, 0.84, 0], 'MarkerEdgeColor', [0.7, 0.55, 0]);
plot(ax4, data.humanoid(1), data.humanoid(2), '^', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', [0.6, 0, 0.6]);
hold(ax4, 'off');

%% Step 9: 全体タイトル
if nnz(data.l1) > 0
    mergedLabel = 'L0+L1+L2';
else
    mergedLabel = 'L0+L2';
end
title(tiled, sprintf('Site transport 20m x 20m - costmaps (%s, %s)', ...
    data.layoutId, mergedLabel), 'FontWeight', 'bold');

%% Step 10: pubfig（FigTools）
if useFigTools && exist('pubfig', 'file') == 2
    pfig = pubfig(hfig);
    pfig.Grid = 'off';
end

%% Step 11: expfig で保存
plotFig_export_figure(hfig, outPath, useFigTools);
fprintf('Saved: %s\n', outPath);
if ~showFigure
    close(hfig);
end

%% Optional: 全レイアウトを連続出力する場合（上の Step 3〜11 をループ）
% for trialIdx = 1:numel(trials)
%     trial = trials(trialIdx);
%     data = plotFig_load_costmap_data(trial.npzPath, trial.trajPath, registryPath);
%     outPath = plotFig_default_output_path(trial.npzPath);
%     % ... Step 4〜11 をここにコピー、またはセクションを関数化せずそのまま貼る
% end
