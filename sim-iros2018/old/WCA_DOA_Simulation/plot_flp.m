function h = plot_flp(flp , option);

%
%   flp                      = load_flp('norwich01.flp');
%   h                        = plot_flp(flp);
%
%

if nargin < 2
   
    option.TX   = 1;
    option.RX   = 1;    
    option.path = 1;
    
else
    
    if (~any(strcmp(fieldnames(option) , 'TX')))

        option.TX = 1;

    end

    if (~any(strcmp(fieldnames(option) , 'RX')))

        option.RX = 1;

    end

    if (~any(strcmp(fieldnames(option) , 'path')))

        option.path = 1;

    end

      
end

h   = gcf;

hold on

if(option.path)
   
    if(any(strcmp(fieldnames(flp) , 'path')))
                      
        currentpath  = 1;
        
        stoploop     = 1;

        while(stoploop ~= length(flp.path) - 2)

            nbpts        = flp.path(currentpath);
            
            stoploop     = currentpath + 5 + (nbpts-1)*5;

            typepath     = flp.path(currentpath + 1);

            xpath        = flp.path(currentpath + 3:5:currentpath + 3 + (nbpts-1)*5);

            ypath        = flp.path(currentpath + 4:5:currentpath + 4 + (nbpts-1)*5);

            zpath        = flp.path(currentpath + 5:5:currentpath + 5 + (nbpts-1)*5);

            if(typepath == 0)
                plot3(xpath , ypath , zpath , 'b');
            end
            
            if(typepath == 1)
                plot3(xpath , ypath , zpath , 'c');
            end
            
            if(typepath == 2)
                plot3(xpath , ypath , zpath , 'g');
            end
                      
            if(typepath == 3)
                plot3(xpath , ypath , zpath , 'r');
            end

            currentpath  = stoploop + 3;

        end
              
    end
    
end

if (option.TX)
    
    plot3(flp.info.TXpoint(1 , :), flp.info.TXpoint(2 , :), flp.info.TXpoint(3 , :), 'ro');

end

if (option.RX)

    plot3(flp.info.RXpoint(1 , :), flp.info.RXpoint(2 , :), flp.info.RXpoint(3 , :), 'y*');

end

xx  = [flp.geom.planes(1,:) ; flp.geom.planes(4,:) ;  flp.geom.planes(7,:) ;  flp.geom.planes(10,:) ;  flp.geom.planes(1,:)];
yy  = [flp.geom.planes(2,:) ; flp.geom.planes(5,:) ;  flp.geom.planes(8,:) ;  flp.geom.planes(11,:) ;  flp.geom.planes(2,:)];
zz  = [flp.geom.planes(3,:) ; flp.geom.planes(6,:) ;  flp.geom.planes(9,:) ;  flp.geom.planes(12,:) ;  flp.geom.planes(3,:)];

plot3(xx , yy , zz, 'k')


view(0,90);
axis equal
hold off


