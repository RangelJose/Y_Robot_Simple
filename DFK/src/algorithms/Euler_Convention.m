function Euler_Convention(Index)
%%Euler Convention
%In this section the Euler convention is selected
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global conLoaded EulerConvention
if isempty(conLoaded)
    conLoaded = true;
    EulerConvention.IndexSaved = Index;
end
   if EulerConvention.IndexSaved ~= Index
        EulerConvention.IndexSaved = Index;
    end

EulerConvention.order = [1 2 3];
EulerConvention.parent= [0 1 2];

switch EulerConvention.IndexSaved
    case 1 %XYZ
        EulerConvention.axes = [1 2 3];
    case 2 %XZY
        EulerConvention.axes = [1 3 2];
    case 3 %XYX
        EulerConvention.axes = [1 2 1];
    case 4 %XZY
        EulerConvention.axes = [1 3 1];
    case 5 %YXZ
        EulerConvention.axes = [2 1 3];
    case 6 %YZX
        EulerConvention.axes = [2 3 1];
    case 7 %YXY
        EulerConvention.axes = [2 1 2];
    case 8 %YZY
        EulerConvention.axes = [2 3 2];
    case 9 %ZXY
        EulerConvention.axes = [3 1 2];
    case 10 %ZYX
        EulerConvention.axes = [3 2 1];
    case 11 %ZXZ
        EulerConvention.axes = [3 1 3];
    case 12 %ZYZ
        EulerConvention.axes = [3 2 3];
end

end