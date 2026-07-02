function S = load_site_transport_npz(npzPath)
%LOAD_SITE_TRANSPORT_NPZ Load numpy .npz costmap archive without Python.
arguments
    npzPath (1, :) char
end

if ~isfile(npzPath)
    error('load_site_transport_npz:MissingFile', 'NPZ not found: %s', npzPath);
end

tmpDir = tempname;
mkdir(tmpDir);
cleanup = onCleanup(@() rmdir(tmpDir, 's')); %#ok<NASGU>

unzip(npzPath, tmpDir);
entries = dir(fullfile(tmpDir, '*.npy'));
if isempty(entries)
    error('load_site_transport_npz:EmptyArchive', 'No .npy arrays in: %s', npzPath);
end

S = struct();
for i = 1:numel(entries)
    [~, name, ~] = fileparts(entries(i).name);
    S.(name) = read_npy_file(fullfile(entries(i).folder, entries(i).name));
end
end

function arr = read_npy_file(npyPath)
fid = fopen(npyPath, 'rb');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

magic = fread(fid, 6, 'uint8=>char')';
if ~strcmp(magic, char([147, 'NUMPY']))
    error('load_site_transport_npz:BadNpy', 'Not a .npy file: %s', npyPath);
end

ver = fread(fid, 2, 'uint8');
if ver(1) == 1
    headerLen = fread(fid, 1, 'uint16');
elseif ver(1) == 2
    headerLen = fread(fid, 1, 'uint32');
else
    error('load_site_transport_npz:BadNpyVersion', 'Unsupported npy version in: %s', npyPath);
end

header = fread(fid, headerLen, 'uint8=>char')';
meta = parse_npy_header(header);
arr = read_npy_payload(fid, meta);
end

function meta = parse_npy_header(header)
meta.descr = parse_descr(header);
meta.fortran_order = parse_fortran_order(header);
meta.shape = parse_shape(header);
end

function descr = parse_descr(header)
tokens = regexp(header, '''descr'': ''([^'']+)''', 'tokens', 'once');
if isempty(tokens)
    error('load_site_transport_npz:BadNpyHeader', 'Missing descr in npy header');
end
descr = tokens{1};
end

function tf = parse_fortran_order(header)
tokens = regexp(header, '''fortran_order'': (True|False)', 'tokens', 'once');
if isempty(tokens)
    error('load_site_transport_npz:BadNpyHeader', 'Missing fortran_order in npy header');
end
tf = strcmp(tokens{1}, 'True');
end

function shape = parse_shape(header)
if ~isempty(regexp(header, '''shape'': \(\)', 'once'))
    shape = 1;
    return;
end
tokens = regexp(header, '''shape'': \(([^)]*)\)', 'tokens', 'once');
if isempty(tokens)
    shape = 1;
    return;
end
shape = str2num(tokens{1}); %#ok<ST2NM>
if isempty(shape)
    shape = 1;
end
end

function arr = read_npy_payload(fid, meta)
count = prod(meta.shape);
if isempty(meta.shape) || isequal(meta.shape, 1)
    count = max(count, 1);
end
switch meta.descr
    case '<f4'
        data = fread(fid, count, 'float32');
    case '<f8'
        data = fread(fid, count, 'float64');
    case '<i4'
        data = fread(fid, count, 'int32');
    case '<i8'
        data = fread(fid, count, 'int64');
    otherwise
        error('load_site_transport_npz:BadNpyDescr', 'Unsupported dtype: %s', meta.descr);
end

if isempty(meta.shape) || isequal(meta.shape, 1)
    arr = double(data(1));
    return;
end
if isscalar(meta.shape)
    arr = double(data(:));
    return;
end

if meta.fortran_order
    arr = reshape(data, meta.shape);
else
    arr = reshape(data, fliplr(meta.shape));
    arr = permute(arr, numel(meta.shape):-1:1);
end
arr = double(arr);
end
