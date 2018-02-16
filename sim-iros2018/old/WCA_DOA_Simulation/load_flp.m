function out = load_flp(filename)

%
%
% flp =  load_flp;
%


detminmax    = 0.001;  % adjustment value for min / max coordinates of planes

if(nargin < 1)

    [out.info.filename, out.info.pathname]    = uigetfile('*.flp', 'Select an flp File');

    [out.info.pathname,out.info.filename,ext] = fileparts([out.info.pathname out.info.filename]);
        
    if isequal(out.info.filename , 0) || isequal(out.info.pathname , 0)

        disp('User pressed cancel')

        return;

    end

else

    out.info.filename                         = filename;
    
    [out.info.pathname,out.info.filename,ext] = fileparts(out.info.filename);
    
end

if(isempty(out.info.filename))

    return;

end


out.info.filename     = fullfile(out.info.pathname, [out.info.filename , ext]);

file_in               = fopen(out.info.filename, 'r');

dum                   = fscanf(file_in, '%s', 2);

out.info.fc           = fscanf(file_in, '%f', 1);

dum                   = fscanf(file_in, '%s', 3);

out.info.flp_scale    = fscanf(file_in, '%f', 1);

dum                   = fscanf(file_in, '%s', 3);

out.info.nplanes      = fscanf(file_in, '%d', 1);

dum                   = fscanf(file_in, '%s', 3);

out.info.nr           = fscanf(file_in, '%d', 1);

dum                   = fscanf(file_in, '%s', 2);

out.info.TXpoint      = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 2);

out.info.RXpoint      = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 3);

out.info.TXantdir     = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 3);

out.info.TXantpol     = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 3);

out.info.TXanttype    = fscanf(file_in, '%d', 1);

dum                   = fscanf(file_in, '%s', 3);

out.info.RXantdir     = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 3);

out.info.RXantpol     = fscanf(file_in, '%d', 3);

dum                   = fscanf(file_in, '%s', 3);

out.info.RXanttype    = fscanf(file_in, '%d', 1);

matrix_in             = zeros(out.info.nplanes , 18);



out.geom.material     = zeros(6 , out.info.nplanes);
out.geom.planes       = zeros(22 , out.info.nplanes );




for i=1:out.info.nplanes
    
    for j=1:18
        matrix_in(i,j) = fscanf(file_in, '%f', 1);
    end

    out.geom.material(1,i)     = matrix_in(i,1); % not used
    out.geom.material(2,i)     = matrix_in(i,2); % not used
    out.geom.material(3,i)     = matrix_in(i,3); % not used
    out.geom.material(4,i)     = matrix_in(i,4); % not used
    out.geom.material(5,i)     = matrix_in(i,5); % Er
    out.geom.material(6,i)     = matrix_in(i,6); % sigma

    pli_x1                     = matrix_in(i,7);
    out.geom.planes(1,i)       = pli_x1; % x of point 1 of floorplan

    pli_y1                     = matrix_in(i,8);
    out.geom.planes(2,i)       = pli_y1; % y of point 1 of floorplan

    pli_z1                     = matrix_in(i,9);
    out.geom.planes(3,i)       = pli_z1; % z of point 1 of floorplan


    pli_x2                     = matrix_in(i,10);
    out.geom.planes(4,i)       = pli_x2; % x of point 2 of floorplan

    pli_y2                     = matrix_in(i,11);
    out.geom.planes(5,i)       = pli_y2; % y of point 2 of floorplan

    pli_z2                     = matrix_in(i,12);
    out.geom.planes(6,i)       = pli_z2; % z of point 2 of floorplan


    pli_x3                     = matrix_in(i,13);
    out.geom.planes(7,i)       = pli_x3; % x of point 3 of floorplan

    pli_y3                     = matrix_in(i,14);
    out.geom.planes(8,i)       = pli_y3; % y of point 3 of floorplan

    pli_z3                     = matrix_in(i,15);
    out.geom.planes(9,i)       = pli_z3; % z of point 3 of floorplan

    pli_x4                     = matrix_in(i,16);
    out.geom.planes(10,i)      = pli_x4; % x of point 4 of floorplan

    pli_y4                     = matrix_in(i,17);
    out.geom.planes(11,i)      = pli_y4; % y of point 4 of floorplan

    pli_z4                     = matrix_in(i,18);
    out.geom.planes(12,i)      = pli_z4; % z of point 4 of floorplan


    pli_norm_x                 = (pli_y1 - pli_y2)*(pli_z1 + pli_z2) + (pli_y2 - pli_y3)*(pli_z2 + pli_z3) + (pli_y3 - pli_y1)*(pli_z3 + pli_z1);
    pli_norm_y                 = (pli_z1 - pli_z2)*(pli_x1 + pli_x2) + (pli_z2 - pli_z3)*(pli_x2 + pli_x3) + (pli_z3 - pli_z1)*(pli_x3 + pli_x1);
    pli_norm_z                 = (pli_x1 - pli_x2)*(pli_y1 + pli_y2) + (pli_x2 - pli_x3)*(pli_y2 + pli_y3) + (pli_x3 - pli_x1)*(pli_y3 + pli_y1);
    mag_vec                    = sqrt(pli_norm_x*pli_norm_x + pli_norm_y*pli_norm_y + pli_norm_z*pli_norm_z);
    pli_norm_x                 = pli_norm_x/mag_vec;
    pli_norm_y                 = pli_norm_y/mag_vec;
    pli_norm_z                 = pli_norm_z/mag_vec;
    distance_origin            = -(pli_x1*pli_norm_x + pli_y1*pli_norm_y + pli_z1*pli_norm_z);

    out.geom.planes(13, i)     = pli_norm_x;
    out.geom.planes(14, i)     = pli_norm_y;
    out.geom.planes(15, i)     = pli_norm_z;
    out.geom.planes(16, i)     = distance_origin;

    vectx                      = [pli_x1 pli_x2 pli_x3 pli_x4];
    xmin                       = min(vectx);
    xmax                       = max(vectx);

    vecty                      = [pli_y1 pli_y2 pli_y3 pli_y4];
    ymin                       = min(vecty);
    ymax                       = max(vecty);

    vectz                      = [pli_z1 pli_z2 pli_z3 pli_z4];
    zmin                       = min(vectz);
    zmax                       = max(vectz);

    if xmin == xmax
        xmin = xmin - detminmax;
        xmax = xmax + detminmax;
    end
    if ymin == ymax
        ymin = ymin - detminmax;
        ymax = ymax + detminmax;
    end
    if zmin == zmax
        zmin = zmin - detminmax;
        zmax = zmax + detminmax;
    end

    out.geom.planes(17, i) = xmin;
    out.geom.planes(18, i) = xmax;
    
    out.geom.planes(19, i) = ymin;
    out.geom.planes(20, i) = ymax;
    
    out.geom.planes(21, i) = zmin;
    out.geom.planes(22, i) = zmax;

end

fclose(file_in);
