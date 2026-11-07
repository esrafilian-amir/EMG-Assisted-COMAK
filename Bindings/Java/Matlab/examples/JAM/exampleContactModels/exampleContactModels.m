%% exampleContactModels
% Runs the walking models (original and with the tuned contact settings) and reports the contact results.
%
% For every model:
%   1. COMAK inverse kinematics + COMAK (the settings of exampleWalking.m / exampleWalkingMeniscus.m, run
%      unchanged apart from the paths) and the joint mechanics tool, which writes the contact vtp files
%      (VTK PolyData: pressure, proximity, ..., and the thickness of the contact meshes) and the forces;
%   2. the contact outputs of every Smith2018ArticularContactForce for every frame of the COMAK states
%      (force, center of pressure, maximum and mean pressure, contact area) -> contact_results.csv;
%   3. the thickness of the contact meshes -> thickness_<mesh>.mat.
% Then one pdf report: contact forces, pressures, centers of pressure, contact areas, the thickness of the
% cartilage and the comparison of the tuned models with the originals.
%
% Results (in the results folder next to this script), one folder per model:
%   results/contact_report.pdf, results/summary.csv
%   results/<model>/inputs, comak-inverse-kinematics, comak, joint-mechanics, contact-analysis, log
%
% Models: lenhart2015 and lenhart2015_contact_tuned (walking, no menisci), smith2019,
% smith2019_contact_tuned and smith2019_contact_tuned_nonlinear (walking with menisci).
% The python version of this example is Bindings/Python/examples/JAM/exampleContactModels.
import org.opensim.modeling.*

%% Options
% (a variable set before running the script keeps its value, e.g. models_to_run = {'lenhart2015'};)
if ~exist('models_to_run', 'var'), models_to_run = {}; end   % {} runs all models, or e.g. {'lenhart2015','lenhart2015_contact_tuned'}
if ~exist('stages', 'var'), stages = 'all'; end          % 'all', 'pipeline', 'analyze' or 'report'
if ~exist('skip_existing', 'var'), skip_existing = false; end  % true: do not run again a model that already has a COMAK states file
if ~exist('png_dir', 'var'), png_dir = ''; end        % a folder: also write every page of the report as a png

%% Folders and models
here = fileparts(mfilename('fullpath'));
examples_dir = fileparts(here);                       % the JAM examples folder
models_dir = fullfile(examples_dir, 'models');
if ~exist('results_dir', 'var') || isempty(results_dir)     % set results_dir before running to write elsewhere
    results_dir = fullfile(here, 'results');
end
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

cases = struct('key', {}, 'example', {}, 'script', {}, 'model', {}, 'title', {}, 'pair', {});
cases(end+1) = struct('key', 'lenhart2015', 'example', 'exampleWalking', 'script', 'exampleWalking.m', ...
    'model', 'healthy/lenhart2015/lenhart2015.osim', 'title', 'Walking, lenhart2015 (original)', 'pair', '');
cases(end+1) = struct('key', 'lenhart2015_contact_tuned', 'example', 'exampleWalking', ...
    'script', 'exampleWalking.m', 'model', 'healthy/lenhart2015/lenhart2015_contact_tuned.osim', ...
    'title', 'Walking, lenhart2015 (tuned contact)', 'pair', 'lenhart2015');
cases(end+1) = struct('key', 'smith2019', 'example', 'exampleWalkingMeniscus', ...
    'script', 'exampleWalkingMeniscus.m', 'model', 'healthy/smith2019/smith2019.osim', ...
    'title', 'Walking with menisci, smith2019 (original, linear)', 'pair', '');
cases(end+1) = struct('key', 'smith2019_contact_tuned', 'example', 'exampleWalkingMeniscus', ...
    'script', 'exampleWalkingMeniscus.m', 'model', 'healthy/smith2019/smith2019_contact_tuned.osim', ...
    'title', 'Walking with menisci, smith2019 (tuned contact, linear)', 'pair', 'smith2019');
cases(end+1) = struct('key', 'smith2019_contact_tuned_nonlinear', 'example', 'exampleWalkingMeniscus', ...
    'script', 'exampleWalkingMeniscus.m', 'model', 'healthy/smith2019/smith2019_contact_tuned_nonlinear.osim', ...
    'title', 'Walking with menisci, smith2019 (tuned contact, nonlinear)', 'pair', 'smith2019');

if ~isempty(models_to_run)
    cases = cases(ismember({cases.key}, models_to_run));
end

%% Run the models
if any(strcmp(stages, {'all', 'pipeline'}))
    for i = 1:numel(cases)
        c = cases(i);
        outdir = fullfile(results_dir, c.key);
        if skip_existing && ~isempty(dir(fullfile(outdir, 'comak', '*_states.sto')))
            continue
        end
        run_pipeline(c, outdir, examples_dir, models_dir);
    end
end

%% Contact results of every frame, thickness of the meshes
if any(strcmp(stages, {'all', 'analyze'}))
    for i = 1:numel(cases)
        analyze_model(cases(i), fullfile(results_dir, cases(i).key), models_dir);
    end
end

%% Report
if any(strcmp(stages, {'all', 'report'}))
    make_report(cases, results_dir, png_dir);
    write_summary(cases, results_dir);
end


%% ------------------------------------------------------------------------
% Local functions
%% ------------------------------------------------------------------------
function run_pipeline(c, outdir, examples_dir, models_dir)
    import org.opensim.modeling.*
    if ~exist(outdir, 'dir'), mkdir(outdir); end
    if ~exist(fullfile(outdir, 'log'), 'dir'), mkdir(fullfile(outdir, 'log')); end
    script = prepare_script(c, outdir, examples_dir, models_dir);
    logfile = fullfile(outdir, 'log', 'pipeline.log');
    Logger.addFileSink(logfile);
    t0 = tic;
    errmsg = '';
    start_dir = pwd;
    try
        execute_script(script);
    catch err
        errmsg = err.message;
    end
    cd(start_dir);
    Logger.removeFileSink();
    wall = toc(t0);
    text = fileread(logfile);
    tm = parse_times(text);
    info = struct('model', c.key, 'wall_seconds', wall, 'ik_s', tm.ik_s, 'jm_s', tm.jm_s, 'comak_s', tm.comak_s, ...
        'all_frames_converged', contains(text, 'All frames converged'), ...
        'errors', numel(regexp(text, '\[error\]')), 'matlab_error', errmsg);
    fid = fopen(fullfile(outdir, 'log', 'pipeline_info.json'), 'w');
    fwrite(fid, jsonencode(info, 'PrettyPrint', true));
    fclose(fid);
    fprintf('%-36s done in %.0f s (converged %d)\n', c.key, wall, info.all_frames_converged);
end

function execute_script(script)
    % a function, so that the variables of the example script stay out of the workspace of this script
    run(script);
end

function script = prepare_script(c, outdir, examples_dir, models_dir)
    % The example script with the model, the paths and the options of this run.
    text = fileread(fullfile(examples_dir, c.example, c.script));
    fwd = @(p) strrep(p, '\', '/');
    % (placeholders, so that no path or code is read as a regexprep escape)
    text = regexprep(text, '^clear;[ \t\r]*$', '', 'lineanchors');
    text = regexprep(text, '^model_file = .*$', 'model_file = @@MODEL@@;', 'lineanchors');
    text = strrep(text, '@@MODEL@@', ['''' fwd(fullfile(models_dir, c.model)) '''']);
    text = strrep(text, '''../models/', ['''' fwd(models_dir) '/']);
    % the result folders of the example (./results/<x>) are subfolders of the folder of this model
    text = strrep(text, '''./results/', '''./');
    text = strrep(text, '''./results''', '''.''');
    text = strrep(text, 'set_use_visualizer(true)', 'set_use_visualizer(false)');
    % the meniscus example has the runs commented out
    text = regexprep(text, '^%\s*(comak_ik\.run\(\);)', '$1', 'lineanchors');
    text = regexprep(text, '^%\s*(comak\.run\(\);)', '$1', 'lineanchors');
    % joint mechanics after COMAK: the contact vtp files with the thickness of the meshes
    jm = strjoin({'comak.set_run_joint_mechanics(true);', ...
        'jm = comak.upd_joint_mechanics();', ...
        'jm.set_contacts(0,''all'');', ...
        'jm.set_contact_outputs(0,''all'');', ...
        'jm.set_contact_mesh_properties(0,''thickness'');', ...
        'jm.set_write_vtp_files(true);', ...
        'jm.set_write_h5_file(false);', ...
        'jm.set_results_directory(''./joint-mechanics'');', ...
        'comak.run();'}, newline);
    assert(~isempty(regexp(text, '^comak\.run\(\);', 'once', 'lineanchors')), ...
        'comak.run() not found in %s', c.script);
    text = regexprep(text, '^comak\.run\(\);', '@@JM@@', 'lineanchors');
    text = strrep(text, '@@JM@@', jm);
    text = [sprintf('cd(''%s'');\n', fwd(outdir)) text];
    script = fullfile(outdir, 'log', 'run_pipeline.m');
    fid = fopen(script, 'w');
    fwrite(fid, text);
    fclose(fid);
end

function analyze_model(c, outdir, models_dir)
    import org.opensim.modeling.*
    ana = fullfile(outdir, 'contact-analysis');
    if ~exist(ana, 'dir'), mkdir(ana); end
    model = Model(fullfile(models_dir, c.model));
    contacts = {};
    fs = model.getForceSet();
    for i = 0:fs.getSize()-1
        f = fs.get(i);
        if strcmp(char(f.getConcreteClassName()), 'Smith2018ArticularContactForce')
            contacts{end+1} = Smith2018ArticularContactForce.safeDownCast(f); %#ok<AGROW>
        end
    end
    model.initSystem();
    st = dir(fullfile(outdir, 'comak', '*_states.sto'));
    st = st(~contains({st.name}, 'settle'));
    assert(~isempty(st), 'no COMAK states file in %s', fullfile(outdir, 'comak'));
    traj = StatesTrajectory.createFromStatesStorage(model, Storage(fullfile(outdir, 'comak', st(1).name)), true, true);
    fields = {'force_x', 'force_y', 'force_z', 'force_mag', 'cop_x', 'cop_y', 'cop_z', ...
        'max_pressure', 'mean_pressure', 'area'};
    header = {'time'};
    for j = 1:numel(contacts)
        for q = 1:numel(fields)
            header{end+1} = [char(contacts{j}.getName()) '__' fields{q}]; %#ok<AGROW>
        end
    end
    nframes = traj.getSize();
    data = zeros(nframes, numel(header));
    for k = 0:nframes-1
        s = traj.get(k);
        model.realizeReport(s);
        row = s.getTime();
        for j = 1:numel(contacts)
            ct = contacts{j};
            fv = OutputVec3.safeDownCast(ct.getOutput('casting_total_contact_force')).getValue(s);
            cv = OutputVec3.safeDownCast(ct.getOutput('casting_total_center_of_pressure')).getValue(s);
            f3 = [fv.get(0) fv.get(1) fv.get(2)];
            row = [row f3 norm(f3) cv.get(0) cv.get(1) cv.get(2) ...
                OutputDouble.safeDownCast(ct.getOutput('casting_total_max_pressure')).getValue(s) ...
                OutputDouble.safeDownCast(ct.getOutput('casting_total_mean_pressure')).getValue(s) ...
                OutputDouble.safeDownCast(ct.getOutput('casting_total_contact_area')).getValue(s)]; %#ok<AGROW>
        end
        data(k+1, :) = row;
    end
    writetable(array2table(data, 'VariableNames', header), fullfile(ana, 'contact_results.csv'));

    settings = struct('contacts', struct(), 'meshes', struct());
    for j = 1:numel(contacts)
        ct = contacts{j};
        settings.contacts.(char(ct.getName())) = struct('method', char(ct.get_contact_method()), ...
            'formulation', char(ct.get_elastic_foundation_formulation()), ...
            'lumped', ct.get_use_lumped_contact_model(), 'max_proximity', ct.get_max_proximity(), ...
            'voxel', ct.get_field_voxel_size(), 'taper', ct.get_field_edge_taper_width());
    end
    cg = model.getContactGeometrySet();
    for i = 0:cg.getSize()-1
        g = cg.get(i);
        if ~strcmp(char(g.getConcreteClassName()), 'Smith2018ContactMesh'), continue; end
        m = Smith2018ContactMesh.safeDownCast(g);
        n = m.getNumFaces();
        thickness = zeros(n, 1);
        centers = zeros(n, 3);
        cen = m.getTriangleCenters();
        for j = 0:n-1
            thickness(j+1) = m.getTriangleThickness(j);
            v = cen.get(j);
            centers(j+1, :) = [v.get(0) v.get(1) v.get(2)];
        end
        name = char(m.getName());
        save(fullfile(ana, ['thickness_' name '.mat']), 'thickness', 'centers');
        settings.meshes.(name) = struct('variable', m.get_use_variable_thickness(), ...
            'smoothing', m.get_thickness_smoothing_iterations(), 'faces', n);
    end
    fid = fopen(fullfile(ana, 'model_settings.json'), 'w');
    fwrite(fid, jsonencode(settings, 'PrettyPrint', true));
    fclose(fid);
    fprintf('%-36s analysis: %d frames, %d contacts\n', c.key, nframes, numel(contacts));
end

function r = load_run(results_dir, key)
    d = fullfile(results_dir, key);
    ana = fullfile(d, 'contact-analysis');
    r = [];
    if ~exist(fullfile(ana, 'contact_results.csv'), 'file'), return; end
    T = readtable(fullfile(ana, 'contact_results.csv'), 'VariableNamingRule', 'preserve');
    r.key = key;
    r.time = T.time;
    r.contacts = struct();
    for v = T.Properties.VariableNames(2:end)
        parts = split(v{1}, '__');
        r.contacts.(parts{1}).(parts{2}) = T.(v{1});
    end
    r.settings = jsondecode(fileread(fullfile(ana, 'model_settings.json')));
    r.info = struct('ik_s', NaN, 'jm_s', NaN, 'comak_s', NaN, 'all_frames_converged', false);
    if exist(fullfile(d, 'log', 'pipeline_info.json'), 'file')
        r.info = jsondecode(fileread(fullfile(d, 'log', 'pipeline_info.json')));
    end
    for fn = {'ik_s', 'jm_s', 'comak_s'}
        if ~isfield(r.info, fn{1}) || isempty(r.info.(fn{1})), r.info.(fn{1}) = NaN; end
    end
    if exist(fullfile(d, 'log', 'pipeline.log'), 'file')    % the times of the tools, from the log
        tm = parse_times(fileread(fullfile(d, 'log', 'pipeline.log')));
        r.info.ik_s = tm.ik_s; r.info.jm_s = tm.jm_s; r.info.comak_s = tm.comak_s;
    end
    r.thickness = struct();
    for f = dir(fullfile(ana, 'thickness_*.mat'))'
        z = load(fullfile(ana, f.name));
        r.thickness.(erase(erase(f.name, 'thickness_'), '.mat')) = z;
    end
end

function write_summary(cases, results_dir)
    rows = {};
    for i = 1:numel(cases)
        r = load_run(results_dir, cases(i).key);
        if isempty(r), continue; end
        row = struct('model', cases(i).key, 'ik_and_sweep_s', r.info.ik_s, 'joint_mechanics_s', r.info.jm_s, ...
            'comak_s', r.info.comak_s, 'all_frames_converged', double(r.info.all_frames_converged));
        for n = fieldnames(r.contacts)'
            c = r.contacts.(n{1});
            row.([n{1} '_peak_force_N']) = max(c.force_mag);
            row.([n{1} '_peak_max_pressure_MPa']) = max(c.max_pressure) / 1e6;
            row.([n{1} '_peak_mean_pressure_MPa']) = max(c.mean_pressure) / 1e6;
            row.([n{1} '_peak_area_mm2']) = max(c.area) * 1e6;
        end
        rows{end+1} = row; %#ok<AGROW>
    end
    if isempty(rows), return; end
    allnames = {};
    for i = 1:numel(rows), allnames = union(allnames, fieldnames(rows{i}), 'stable'); end
    % plain csv
    fid = fopen(fullfile(results_dir, 'summary.csv'), 'w');
    fprintf(fid, '%s\n', strjoin(allnames', ','));
    for i = 1:numel(rows)
        vals = cell(1, numel(allnames));
        for j = 1:numel(allnames)
            if isfield(rows{i}, allnames{j}), v = rows{i}.(allnames{j}); else, v = NaN; end
            if ischar(v), vals{j} = v; elseif isnan(v), vals{j} = ''; else, vals{j} = sprintf('%.4g', v); end
        end
        fprintf(fid, '%s\n', strjoin(vals, ','));
    end
    fclose(fid);
end

function tm = parse_times(text)
    % Seconds of the IK (with the sweep), the joint mechanics and COMAK (without the joint mechanics), from the
    % Finished in line after the '<tool> complete' line of the log.
    tm = struct('ik_s', NaN, 'jm_s', NaN, 'comak_s', NaN);
    tok = regexp(text, '(COMAKInverseKinematics|JointMechanicsTool|COMAK) complete.s*[info] Finished in (d+) second', 'tokens');
    for i = 1:numel(tok)
        v = str2double(tok{i}{2});
        switch tok{i}{1}
            case 'COMAKInverseKinematics', tm.ik_s = v;
            case 'JointMechanicsTool', tm.jm_s = v;
            case 'COMAK', tm.comak_s = v;
        end
    end
    if ~isnan(tm.comak_s) && ~isnan(tm.jm_s), tm.comak_s = tm.comak_s - tm.jm_s; end
end

function make_report(cases, results_dir, png_dir)
    pdf = fullfile(results_dir, 'contact_report.pdf');
    if exist(pdf, 'file'), delete(pdf); end
    runs = struct();
    keys = {};
    for i = 1:numel(cases)
        r = load_run(results_dir, cases(i).key);
        if ~isempty(r), runs.(cases(i).key) = r; keys{end+1} = cases(i).key; end %#ok<AGROW>
    end
    titles = containers.Map({cases.key}, {cases.title});
    pairs = containers.Map({cases.key}, {cases.pair});
    npage = 0;

    function newpage(ttl, sub)
        fig = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 11.69 8.27], 'Color', 'w');
        annotation(fig, 'textbox', [0.02 0.94 0.96 0.05], 'String', ttl, 'FontSize', 13, ...
            'FontWeight', 'bold', 'EdgeColor', 'none', 'Interpreter', 'none');
        if nargin > 1
            annotation(fig, 'textbox', [0.02 0.905 0.96 0.04], 'String', sub, 'FontSize', 8, ...
                'Color', [0.35 0.35 0.35], 'EdgeColor', 'none', 'Interpreter', 'none');
        end
        figs = fig; %#ok<NASGU>
        set(0, 'CurrentFigure', fig);
    end

    function savepage(fig, ctype)
        npage = npage + 1;
        exportgraphics(fig, pdf, 'ContentType', ctype, 'Resolution', 200, 'Append', npage > 1);
        if ~isempty(png_dir)
            if ~exist(png_dir, 'dir'), mkdir(png_dir); end
            exportgraphics(fig, fullfile(png_dir, sprintf('page_%02d.png', npage)), 'Resolution', 80);
        end
        close(fig);
    end

    % ---- summary
    newpage('Contact model report: walking trial (COMAK, joint mechanics)', ...
        'Smith2018ArticularContactForce. Time window 1.28 to 1.91 s of the walking trial. Generated by exampleContactModels.m');
    fig = gcf;
    lines = {sprintf('%-36s %-26s %-9s %-9s %-9s %-9s %-10s %-10s %-10s %-10s', 'model', 'contact method', ...
        'smoothing', 'IK (s)', 'COMAK (s)', 'converged', 'tf F (N)', 'tf P (MPa)', 'pf F (N)', 'pf P (MPa)')};
    for k = 1:numel(keys)
        r = runs.(keys{k});
        nm = fieldnames(r.settings.contacts);
        s1 = r.settings.contacts.(nm{1});
        sm = [];
        for m = fieldnames(r.settings.meshes)'
            if r.settings.meshes.(m{1}).variable, sm(end+1) = r.settings.meshes.(m{1}).smoothing; end %#ok<AGROW>
        end
        tf = ''; tfp = ''; pf = ''; pfp = '';
        if isfield(r.contacts, 'tf_contact')
            tf = sprintf('%.0f', max(r.contacts.tf_contact.force_mag));
            tfp = sprintf('%.2f', max(r.contacts.tf_contact.max_pressure) / 1e6);
        end
        if isfield(r.contacts, 'pf_contact')
            pf = sprintf('%.0f', max(r.contacts.pf_contact.force_mag));
            pfp = sprintf('%.2f', max(r.contacts.pf_contact.max_pressure) / 1e6);
        end
        lines{end+1} = sprintf('%-36s %-26s %-9s %-9s %-9s %-9s %-10s %-10s %-10s %-10s', keys{k}, ...
            [s1.method '/' s1.formulation], num2str(unique(sm)), num2str(r.info.ik_s), ...
            num2str(r.info.comak_s), char(string(r.info.all_frames_converged)), tf, tfp, pf, pfp); %#ok<AGROW>
    end
    annotation(fig, 'textbox', [0.02 0.45 0.96 0.4], 'String', lines, 'FontName', 'FixedWidth', ...
        'FontSize', 6.5, 'EdgeColor', 'none', 'Interpreter', 'none');
    savepage(fig, 'vector');

    % ---- per model: one row per contact
    for k = 1:numel(keys)
        r = runs.(keys{k});
        names = fieldnames(r.contacts);
        per_page = 3;
        for p0 = 1:per_page:numel(names)
            chunk = names(p0:min(p0 + per_page - 1, numel(names)));
            newpage(titles(keys{k}), 'contact force (components and magnitude), pressure, center of pressure, contact area');
            fig = gcf;
            tl = tiledlayout(fig, numel(chunk), 4, 'Padding', 'compact', 'TileSpacing', 'compact');
            tl.OuterPosition = [0.02 0.03 0.96 0.86];
            for i = 1:numel(chunk)
                c = r.contacts.(chunk{i});
                t = r.time;
                ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
                plot(ax, t, c.force_x, 'LineWidth', 0.8); plot(ax, t, c.force_y, 'LineWidth', 0.8);
                plot(ax, t, c.force_z, 'LineWidth', 0.8); plot(ax, t, c.force_mag, 'k', 'LineWidth', 1.4);
                title(ax, [chunk{i} ': contact force (N)'], 'Interpreter', 'none', 'FontSize', 8);
                legend(ax, {'x', 'y', 'z', '|F|'}, 'Box', 'off', 'FontSize', 6, 'Orientation', 'horizontal');
                ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
                plot(ax, t, c.max_pressure / 1e6, 'Color', [0.84 0.15 0.16], 'LineWidth', 1.2);
                plot(ax, t, c.mean_pressure / 1e6, 'Color', [0.12 0.47 0.71], 'LineWidth', 1.2);
                title(ax, 'pressure (MPa)', 'FontSize', 8); legend(ax, {'max', 'mean'}, 'Box', 'off', 'FontSize', 6);
                ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
                plot(ax, t, c.cop_x * 1000, 'LineWidth', 1); plot(ax, t, c.cop_y * 1000, 'LineWidth', 1);
                plot(ax, t, c.cop_z * 1000, 'LineWidth', 1);
                title(ax, 'center of pressure (mm)', 'FontSize', 8);
                legend(ax, {'x', 'y', 'z'}, 'Box', 'off', 'FontSize', 6, 'Orientation', 'horizontal');
                ax = nexttile(tl); grid(ax, 'on');
                plot(ax, t, c.area * 1e6, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2);
                title(ax, 'contact area (mm^2)', 'FontSize', 8);
                if i == numel(chunk), xlabel(ax, 'time (s)'); end
            end
            savepage(fig, 'vector');
        end
    end

    % ---- comparisons: tuned against original
    for k = 1:numel(keys)
        ref_key = pairs(keys{k});
        if isempty(ref_key) || ~isfield(runs, ref_key), continue; end
        r = runs.(keys{k}); ref = runs.(ref_key);
        names = intersect(fieldnames(r.contacts), fieldnames(ref.contacts), 'stable');
        newpage([keys{k} ' against ' ref_key], ...
            'contact force magnitude, maximum pressure and center of pressure distance; deviations relative to the trial peak of the original');
        fig = gcf;
        tl = tiledlayout(fig, numel(names), 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        tl.OuterPosition = [0.02 0.14 0.96 0.75];
        txt = {};
        for i = 1:numel(names)
            c = r.contacts.(names{i}); c0 = ref.contacts.(names{i});
            ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
            plot(ax, ref.time, c0.force_mag, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4);
            plot(ax, r.time, c.force_mag, 'Color', [0.84 0.15 0.16], 'LineWidth', 1);
            title(ax, [names{i} ': force |F| (N)'], 'Interpreter', 'none', 'FontSize', 8);
            if i == 1, legend(ax, {'original', 'tuned'}, 'Box', 'off', 'FontSize', 6); end
            ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
            plot(ax, ref.time, c0.max_pressure / 1e6, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4);
            plot(ax, r.time, c.max_pressure / 1e6, 'Color', [0.84 0.15 0.16], 'LineWidth', 1);
            title(ax, 'max pressure (MPa)', 'FontSize', 8);
            m = min(numel(r.time), numel(ref.time));
            d = sqrt((c.cop_x(1:m) - c0.cop_x(1:m)).^2 + (c.cop_y(1:m) - c0.cop_y(1:m)).^2 + ...
                (c.cop_z(1:m) - c0.cop_z(1:m)).^2) * 1000;
            ax = nexttile(tl); grid(ax, 'on');
            plot(ax, r.time(1:m), d, 'Color', [0.17 0.63 0.17], 'LineWidth', 1);
            title(ax, 'center of pressure distance (mm)', 'FontSize', 8);
            fpk = max(c0.force_mag); ppk = max(c0.max_pressure);
            use = c0.force_mag(1:m) > 0.1 * fpk;
            txt{end+1} = sprintf(['%s: peak force %+.1f%%, RMS %.1f%%; peak max pressure %+.1f%%, RMS %.1f%%; ' ...
                'COP mean %.2f mm (frames above 10%% of peak force)'], names{i}, ...
                100 * (max(c.force_mag) - fpk) / fpk, 100 * sqrt(mean((c.force_mag(1:m) - c0.force_mag(1:m)).^2)) / fpk, ...
                100 * (max(c.max_pressure) - ppk) / ppk, 100 * sqrt(mean((c.max_pressure(1:m) - c0.max_pressure(1:m)).^2)) / ppk, ...
                mean(d(use))); %#ok<AGROW>
        end
        annotation(fig, 'textbox', [0.02 0.01 0.96 0.12], 'String', txt, 'FontName', 'FixedWidth', ...
            'FontSize', 6.5, 'EdgeColor', 'none', 'Interpreter', 'none');
        savepage(fig, 'vector');
    end

    % ---- thickness
    for k = 1:numel(keys)
        r = runs.(keys{k});
        ref_key = pairs(keys{k});
        has_ref = ~isempty(ref_key) && isfield(runs, ref_key);
        meshes = {};
        for m = fieldnames(r.settings.meshes)'
            if r.settings.meshes.(m{1}).variable, meshes{end+1} = m{1}; end %#ok<AGROW>
        end
        if isempty(meshes), continue; end
        newpage(['Cartilage thickness: ' titles(keys{k})], ...
            'thickness of the contact meshes (measured by a ray to the bone mesh; smoothing = iterations of Laplacian smoothing over the neighbor triangles)');
        fig = gcf;
        tl = tiledlayout(fig, numel(meshes), 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        tl.OuterPosition = [0.02 0.03 0.96 0.86];
        for i = 1:numel(meshes)
            z = r.thickness.(meshes{i});
            [x, y] = plane_projection(z.centers);
            ax = nexttile(tl);
            scatter(ax, x * 1000, y * 1000, 2, z.thickness * 1000, 'filled'); axis(ax, 'equal'); colorbar(ax);
            clim_(ax, [1 5]);
            title(ax, sprintf('%s, smoothing %d (mm)', meshes{i}, r.settings.meshes.(meshes{i}).smoothing), ...
                'Interpreter', 'none', 'FontSize', 8);
            ax = nexttile(tl);
            if has_ref && isfield(runs.(ref_key).thickness, meshes{i})
                z0 = runs.(ref_key).thickness.(meshes{i});
                [x0, y0] = plane_projection(z0.centers);
                scatter(ax, x0 * 1000, y0 * 1000, 2, z0.thickness * 1000, 'filled'); axis(ax, 'equal'); colorbar(ax);
                clim_(ax, [1 5]);
                title(ax, sprintf('%s, smoothing %d (original)', meshes{i}, ...
                    runs.(ref_key).settings.meshes.(meshes{i}).smoothing), 'Interpreter', 'none', 'FontSize', 8);
            else
                axis(ax, 'off');
            end
            ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
            edges = linspace(0.5, max(z.thickness) * 1000 + 0.5, 60);
            histogram(ax, z.thickness * 1000, edges, 'FaceColor', [0.84 0.15 0.16], 'FaceAlpha', 0.6);
            leg = {'this model'};
            if has_ref && isfield(runs.(ref_key).thickness, meshes{i})
                histogram(ax, runs.(ref_key).thickness.(meshes{i}).thickness * 1000, edges, 'DisplayStyle', 'stairs', ...
                    'EdgeColor', [0.2 0.2 0.2]);
                leg{end+1} = 'original'; %#ok<AGROW>
            end
            title(ax, sprintf('thickness histogram: mean %.2f, std %.2f mm', mean(z.thickness) * 1000, ...
                std(z.thickness) * 1000), 'FontSize', 8);
            xlabel(ax, 'thickness (mm)'); legend(ax, leg, 'Box', 'off', 'FontSize', 6);
        end
        savepage(fig, 'image');
    end

    % ---- notes
    newpage('Notes');
    annotation(gcf, 'textbox', [0.03 0.1 0.94 0.8], 'EdgeColor', 'none', 'FontName', 'FixedWidth', 'FontSize', 8, ...
        'Interpreter', 'none', 'String', { ...
        'Contact force: total force of the casting mesh (tibia or patella cartilage; for the menisci the casting mesh of the contact) in N.', ...
        'Pressure: maximum and mean over the contacting triangles (MPa). Contact area in mm^2.', ...
        'Center of pressure: as reported by the contact output total_center_of_pressure (m, plotted in mm).', ...
        'Thickness: per triangle thickness of the variable thickness cartilage meshes; thickness_smoothing_iterations averages it', ...
        '  over the edge neighbors (graph Laplacian), which removes the triangle to triangle jumps of the measured thickness.', ...
        'distance_field: the depth of a casting triangle is read from a signed distance grid of the target mesh (see', ...
        '  scripts/contact_tuning/RESULTS.md for the comparison with the ray cast and the recommended settings).', ...
        'Files per model: <model>/comak (states, activations, forces), <model>/joint-mechanics (contact vtp files with pressure', ...
        '  and thickness for ParaView, forces .sto), <model>/contact-analysis (contact_results.csv, thickness_*.mat), <model>/inputs, <model>/log.'});
    savepage(gcf, 'vector');
    fprintf('report: %s\n', pdf);
end

function [x, y] = plane_projection(xyz)
    c = xyz - mean(xyz, 1);
    [~, ~, V] = svd(c, 'econ');
    x = c * V(:, 1);
    y = c * V(:, 2);
end

function clim_(ax, lim)
    try
        clim(ax, lim);          % R2022a and later
    catch
        caxis(ax, lim);
    end
end
