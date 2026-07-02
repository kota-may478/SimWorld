function plotFig_export_figure(hfig, outPath, useFigTools)
%PLOTFIG_EXPORT_FIGURE Save figure via FigTools expfig or exportgraphics fallback.

[outDir, baseName, ext] = fileparts(outPath);
if isempty(outDir)
    outDir = pwd;
end
if isempty(ext)
    outPath = [outPath, '.png'];
    ext = '.png';
end
if ~isempty(outDir) && ~isfolder(outDir)
    mkdir(outDir);
end

exported = false;
if useFigTools && exist('expfig', 'file') == 2
    oldPwd = pwd;
    c = onCleanup(@() cd(oldPwd)); %#ok<NASGU>
    cd(outDir);
    try
        expfig(baseName, '-png', hfig);
        candidates = {
            fullfile(outDir, 'png', [baseName, ext]), ...
            fullfile(outDir, 'plot', 'png', [baseName, ext]), ...
            fullfile(outDir, [baseName, ext])};
        for i = 1:numel(candidates)
            if isfile(candidates{i})
                if ~strcmp(candidates{i}, outPath)
                    movefile(candidates{i}, outPath, 'f');
                end
                exported = true;
                break;
            end
        end
        if isfolder(fullfile(outDir, 'plot'))
            rmdir(fullfile(outDir, 'plot'), 's');
        elseif isfolder(fullfile(outDir, 'png'))
            rmdir(fullfile(outDir, 'png'), 's');
        end
    catch
        exported = false;
    end
end

if ~exported || ~isfile(outPath)
    exportgraphics(hfig, outPath, 'Resolution', 150);
end
end
