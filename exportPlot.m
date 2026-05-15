function exportPlot(hObj, args)
arguments
    hObj (1,1)
    args.Folder string {mustBeScalarOrEmpty, mustBeFolder} = string.empty()
    args.NamePrefix string {mustBeScalarOrEmpty} = string.empty()
end
assert(isgraphics(hObj));
if isa(hObj.Children, "matlab.ui.container.TabGroup")
    for thisTab = hObj.Children.Children(:).'
        exportPlot(thisTab, Folder=args.Folder, NamePrefix=hObj.Name);
    end
    
    return
end

if isa(hObj, "matlab.ui.container.Tab")
    name = hObj.Title;
else
    name = hObj.Name;
end
name = name + ".eps";

if ~isempty(args.NamePrefix)
    name = args.NamePrefix + " - " + name;
end

if isempty(args.Folder)
    filePath = name;
else
    filePath = fullfile(args.Folder, name);
end

fprintf(1, "Exporting %s\n", filePath);
exportgraphics(hObj, filePath, Resolution=300);
end

