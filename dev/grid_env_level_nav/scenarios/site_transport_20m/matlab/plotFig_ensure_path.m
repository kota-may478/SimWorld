function plotFig_ensure_path()
%PLOTFIG_ENSURE_PATH Add this matlab folder to the MATLAB path once.

thisDir = fileparts(mfilename('fullpath'));
if ~contains(path, thisDir)
    addpath(thisDir);
end
end
