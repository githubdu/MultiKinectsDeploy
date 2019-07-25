function MultiKinectsDeploy

clc

%% Figure
InitialRoomSize = 15;
roomSize = InitialRoomSize;
myFig = figure('Color','w','Resize','on','MenuBar','none',...                               
    'NumberTitle','off','Name','ViewOptimazing',...
    'units','normalized','Position',[0.2 0.2 0.6 0.6],...
    'WindowButtonUpFcn',@(s,e)myWindowButtonUpFcn,...
    'WindowButtonMotionFcn',@(s,e)myMotionFcn,...
    'WindowScrollWheelFcn',@myScrollWheelFcn);   
myAxes = axes('units','normalized','Position',[0 0 1 1],...
    'Parent',myFig,'XLim',[-roomSize*1.33 roomSize*3],...
    'YLim',[-roomSize*1.2 roomSize*1.2],...
    'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
hold(myAxes,'on')
axis(myAxes,'off')

%% Walls
walls(1).initialShape = [roomSize roomSize -roomSize -roomSize roomSize;...
                         roomSize -roomSize -roomSize roomSize roomSize];
walls(2).initialShape = [28.5 28.5 12 12 28.5; 10 0 0 10 10]*roomSize/10;
walls(3).initialShape = [28.5 28.5 12 12 28.5;...
                        -1.5 -10 -10 -1.5 -1.5]*roomSize/10;
walls(4).initialShape = [26 26 14 14 26; 8.5 1.5 1.5 8.5 8.5]*roomSize/10;
for ii = 1:length(walls)
    walls(ii).shape = walls(ii).initialShape;
    if (ii) == 1
        walls(ii).plot = patch('XData',walls(ii).shape(1,:),...
                           'YData',walls(ii).shape(2,:),...
                           'FaceColor','k','Parent',myAxes,...
                           'FaceAlpha',0.3,...
                           'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
    else
        walls(ii).plot = patch('XData',walls(ii).shape(1,:),...
                               'YData',walls(ii).shape(2,:),...
                               'FaceColor','w','Parent',myAxes,...
                               'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
    end
     walls(ii).coord = [0;0];
end

%% Sensor Objects
% 角度正方向：俯视顺时针方向为正方向
sensor(1:3) = struct;
tempN = length(sensor);
tempV = 0.02+0:2*pi/tempN:0.02+2*pi-1e-10;
tempC = [roomSize*0.9*sin(tempV);roomSize*0.9*cos(tempV)];
for ii = 1:length(sensor)
    sensor(ii).coord = tempC(:,ii); % x, y
    sensor(ii).viewRange = 57/180*pi;
    sensor(ii).viewDirection = tempV(ii)+0.5*pi;
    sensor(ii).viewNear = 0.5;
    sensor(ii).viewFar = 20;
    
    % sensor shape
    transform = rotz(-sensor(ii).viewDirection*180/pi);
    transform(1:2,3) = sensor(ii).coord(1:2)/1000;
    origin_shape = [[0.5 0.5 -0.5 -0.5;1.5 -1.5 -1 1]/2; 1 1 1 1];
    sensor(ii).shape = transform*origin_shape;
    sensor(ii).color = [0 0 1; 0.8 0.8 0.1];
    
    % 绘制传感器
    sensor(ii).plotSensor = ...
    patch('XData',sensor(ii).shape(1,:)+sensor(ii).coord(1),...
          'YData',sensor(ii).shape(2,:)+sensor(ii).coord(2),...
          'FaceColor',sensor(ii).color(1,:),'Parent',myAxes,'FaceAlpha',...
          0.9,'ButtonDownFcn',@(s,e)myButtonDownFcn('sensor',ii));
      
    % sensor view area
    n = 20;  % 用线段拟合传感器内外视界的弧线
    t1 = linspace(-sensor(ii).viewRange/2, sensor(ii).viewRange/2,n);
    t2 = linspace( sensor(ii).viewRange/2,-sensor(ii).viewRange/2,n);
    sensor(ii).viewArea = [cos(t1)*sensor(ii).viewNear,...
                            cos(t2)*sensor(ii).viewFar; ...
                            sin(t1)*sensor(ii).viewNear,...
                            sin(t2)*sensor(ii).viewFar; ...
                            linspace(1,1,n) linspace(1,1,n)];
    transform = rotz(-sensor(ii).viewDirection*180/pi);
    transform(1:2,3) = sensor(ii).coord(1:2);  
    sensor(ii).viewArea = transform*sensor(ii).viewArea;
      
    % 绘制传感器视野
    [f1,v1] = poly2fv(sensor(ii).viewArea(1,:),sensor(ii).viewArea(2,:));
    sensor(ii).plotViewArea = patch('Faces',f1,'Vertices',v1,...
        'FaceColor','g','Parent',myAxes,'EdgeColor','none','FaceAlpha',0.5);
    
    % 两个相邻传感器之间的公共视野
    sensor(ii).interArea = 0;
    if ii > 1
        vax = sensor(ii-1).viewArea(1,:);
        vay = sensor(ii-1).viewArea(2,:);
        if (~ispolycw(vax,vay))
            [vax,vay] = poly2cw(vax,vay);
        end
        [intx,inty] = polybool('and',sensor(ii).viewArea(1,:),...
            sensor(ii).viewArea(2,:),vax,vay);
        sensor(ii).interArea = (polyarea(intx,inty));
    else
        intx = []; inty = [];
    end
    [tf1, tv1] = poly2fv(intx, inty);
    sensor(ii).plotInterArea = patch('Faces',tf1,'Vertices',tv1,...
        'FaceColor','r','Parent',myAxes,'EdgeColor','none','FaceAlpha',0.5);
end
%% obstacle distance 
obstacle.plotUnkown = patch('Faces',[],'Vertices',[],...
    'FaceColor','r','Parent',myAxes,'EdgeColor','none','FaceAlpha',0.9);

obstacle.plotMiniDis = plot(0,0,'r','lineWidth',3,'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
obstacle.dispalyDis = text(0,0,'','Parent',myAxes,'HorizontalAlignment',...
    'left','Interpreter','Latex','Color','r','FontSize',14,...
        'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
%% Other Objects
objs(1:2) = struct;
for ii = 1:length(objs)
    objs(ii).coord = [(1-2*rand)*(InitialRoomSize-2); (1-2*rand)*(InitialRoomSize-2)];
    
    if ii < 3
        switch(ii)
            case 1
                objs(ii).viewDirection = pi/length(sensor);
                robotShape = [0.5 1 1 -1 -1 -0.5 0.5;4 3 -3 -3 3 4 4]/2;
                objs(ii).originShape = [robotShape;1 1 1 1 1 1 1];
            case 2
                objs(ii).viewDirection = 0;
                t = linspace(-pi,pi,n);
                objs(ii).originShape = [sin(t);cos(t);ones(1,n)];
        end
        transform = rotz(-objs(ii).viewDirection*180/pi);
        transform(1:2,3) = objs(ii).coord(1:2)/1000;
        objs(ii).shape = transform*objs(ii).originShape;
        objs(ii).tcp = transform*[0;4;1];
    else
        objs(ii).shape = (rand(2,5)*2).*[1 1 -1 -1 1; 1 -1 -1 1 1];
        objs(ii).shape(:,5) = objs(ii).shape(:,1);
    end
    objs(ii).color = [rand*.2 rand*0.2 rand*0.2];
    objs(ii).plot = patch('XData',objs(ii).shape(1,:)+objs(ii).coord(1),'YData',objs(ii).shape(2,:)+objs(ii).coord(2),...
        'FaceColor',objs(ii).color,'Parent',myAxes,...
        'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',ii));
end

%% Control Flags and Variables
myControl.buttonDownFlag = 0;
myControl.type = 'start';
myControl.index = 0;
myMotionFcn
myControl.type = '';

%% UI-Controls
scrollWheelButtonGroup = uibuttongroup('Position',[0.64 0.58 0.26 0.26],'Parent',myFig,'BackgroundColor','w','BorderType','none');
createUIcontrol('radiobutton',[0.05 0.775 0.9 0.15],'Radius',12,'k',scrollWheelButtonGroup,'on','','w');
createUIcontrol('radiobutton',[0.05 0.45 0.9 0.15],'View Range',12,'k',scrollWheelButtonGroup,'on','','w');
createUIcontrol('radiobutton',[0.05 0.1 0.9 0.15],'Room Size',12,'k',scrollWheelButtonGroup,'on','','w');

editBox(1) = createUIcontrol('edit',[0.79 0.775 0.1 0.05],num2str(sensor(1).viewFar),12,'k',myFig,'on',@(s,e)editchange(1),'w');
editBox(2) = createUIcontrol('edit',[0.79 0.685 0.1 0.05],num2str(sensor(1).viewRange),12,'k',myFig,'on',@(s,e)editchange(2),'w');
editBox(3) = createUIcontrol('edit',[0.79 0.595 0.1 0.05],num2str(roomSize),12,'k',myFig,'on',@(s,e)editchange(3),'w');
myManual = sprintf(['\n\n'...
    'Ball: simulated object \n'...
    'Cylinder: simulated robot \n\n'...
    'Controls: \n Left Mouseclick: New Object \n '...
    'Left Mouseclick on Object: Move Object\n ' ...
    'Right Mouseclick on Object: Delete Object \n '...
    'Scrollwheel while clicked on Object: Change Size \n '...
    'Scrollwheel: Depends on Radiobutton']);
manualAxes = axes('units','normalized','Position',[0.6 0.15 0.3 0.3], 'Parent',myFig);
hold(manualAxes,'on')
axis(manualAxes,'off')
text(0,0.5,myManual,'Parent',manualAxes,'HorizontalAlignment','left','Interpreter','Latex','Color','k','FontSize',10,...
        'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);

%% Nested Functions
    function myButtonDownFcn(type,index)
        switch myFig.SelectionType
            case {'normal','open'}
                myControl.buttonDownFlag = 1;
                myControl.type = type;
                myControl.index = index;
            case 'alt'
                if strcmp(type,'obj') && length(objs) > 2
                    delete(objs(index).plot)
                    objs(index) = [];
                    for kk = 1:length(objs)
                        set(objs(kk).plot,'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',kk))
                    end
                    myControl.type = 'start';
                    myMotionFcn
                    myControl.type = '';
                end
        end
    end

    function myWindowButtonUpFcn
        myControl.buttonDownFlag = 0;
        myControl.type = '';
        myControl.index = 0;
    end

    function myMotionFcn
        if strcmp(myControl.type,'start') || (myControl.buttonDownFlag ...
                && ~isempty(myControl.type) && myControl.index > 0)
            C = myAxes.CurrentPoint(1,1:2);
            switch myControl.type
                case 'obj'
                    objs(myControl.index).coord = C';
                    set(objs(myControl.index).plot,'XData',...
                        objs(myControl.index).shape(1,:)+C(1),...
                        'YData',objs(myControl.index).shape(2,:)+C(2))
                case 'sensor'
                    sensor(myControl.index).coord(1:2,1) = C';
                otherwise
            end
            % Create view area
            for kk = 1:length(sensor)
                % sensor view area
                t1 = linspace(-sensor(kk).viewRange/2, sensor(kk).viewRange/2,n);
                t2 = linspace( sensor(kk).viewRange/2,-sensor(kk).viewRange/2,n);
                sensor(kk).viewArea = [cos(t1)*sensor(kk).viewNear,...
                                        cos(t2)*sensor(kk).viewFar; ...
                                        sin(t1)*sensor(kk).viewNear,...
                                        sin(t2)*sensor(kk).viewFar; ...
                                        linspace(1,1,n) linspace(1,1,n)];
                transform = rotz(-sensor(kk).viewDirection*180/pi);
                transform(1:2,3) = sensor(kk).coord(1:2);  
                sensor(kk).viewArea = transform*sensor(kk).viewArea;
    
                % find intersections between sensor view
                for jj = 1:length(sensor)
                    if jj ~=kk
                        updateSensorViews(kk,sensor(jj));
                    end
                end
                
                for jj = 1:length(objs)
                    updateSensorViews(kk,objs(jj));
                end
                
                for jj = 1:length(walls)
                    if inpolygon(sensor(kk).coord(1),sensor(kk).coord(2),...
                            walls(jj).shape(1,:),walls(jj).shape(2,:))
                        if (isempty(sensor(kk).viewArea))
                            break;
                        end
                        nx = sensor(kk).viewArea(1,:);
                        ny = sensor(kk).viewArea(2,:);
                        wx = walls(jj).shape(1,:);
                        wy = walls(jj).shape(2,:);
                        if (~ispolycw(wx,wy))
                            [wx,wy] = poly2cw(wx,wy);
                        end
                        [nx,ny] = polybool('and',nx,ny,wx,wy);
                        sensor(kk).viewArea = [nx;ny];
                    end
                end
                
                if (~isempty(sensor(kk).viewArea))
                    [f,v] = poly2fv(sensor(kk).viewArea(1,:),sensor(kk).viewArea(2,:));
                    set(sensor(kk).plotViewArea,'Faces',f,'Vertices',v);
                end
                
                set(sensor(kk).plotSensor,'XData',sensor(kk).shape(1,:)+sensor(kk).coord(1),...
                                          'YData',sensor(kk).shape(2,:)+sensor(kk).coord(2));
                
                % 两个相邻传感器之间的公共视野
                sensor(kk).interArea = 0;
                if kk > 1
                    intx = []; inty = [];
                    for mm = 1:kk-1
                        if (isempty(sensor(mm).viewArea) || isempty(sensor(kk).viewArea))
                            break;
                        end
                        vax = sensor(mm).viewArea(1,:);
                        vay = sensor(mm).viewArea(2,:);
                        if (~ispolycw(vax,vay))
                            [vax,vay] = poly2cw(vax,vay);
                        end
                        [innerx,innery] = polybool('and',sensor(kk).viewArea(1,:),...
                            sensor(kk).viewArea(2,:),vax,vay);
                        [intx,inty] = polybool('union',intx,inty,innerx,innery);
                    end
                    if (~isempty(intx) && ~isempty(inty))
                        [tf, tv] = poly2fv(intx, inty);
                        set(sensor(kk).plotInterArea, 'Faces',tf,'Vertices',tv);
                    else
                        set(sensor(kk).plotInterArea, 'Faces',[],'Vertices',[]);
                    end
                end
            end 
            
            % 求最短距离
            allx = []; ally = [];
            for kk = 1:length(sensor)
                if(isempty(sensor(kk).viewArea))
                    continue;
                end
                sx = sensor(kk).viewArea(1,:);
                sy = sensor(kk).viewArea(2,:);
                if(~ispolycw(sx,sy))
                    [sx,sy] = poly2cw(sx,sy);
                end
                [allx,ally] = polybool('union',allx,ally,sx,sy);
                
                if(~ispolycw(allx,ally))
                    [allx,ally] = poly2cw(allx,ally);
                end
            end
            
            Shape = [robotShape*1.1;1 1 1 1 1 1 1];
            transform = zeros(3);
            transform = rotz(-objs(1).viewDirection*180/pi);
            transform(1:2,3) = objs(1).coord(1:2);
            shape = transform*Shape;
            transform(1:2,3) = objs(1).coord(1:2);
            tcp = transform*[(Shape(1:2,1)+Shape(1:2,end-1))/2;1];
        
            for kk = 1:length(walls)
                if inpolygon(sensor(1).coord(1),sensor(1).coord(2),...
                            walls(kk).shape(1,:),walls(kk).shape(2,:))
                    wx = walls(kk).shape(1,:);
                    wy = walls(kk).shape(2,:);
                    if(~ispolycw(wx,wy))
                        [wx,wy] = poly2cw(wx,wy);
                    end
                    [allx,ally] = polybool('-',wx,wy,allx,ally);
                    rx = shape(1,:);
                    ry = shape(2,:);
                    if(~ispolycw(rx,ry))
                        [rx,ry] = poly2cw(rx,ry);
                    end
                    [allx,ally] = polybool('-',allx,ally,rx,ry);
                    break;
                end
            end

            [allx,ally]=polymerge(allx,ally);
            
            [d,x_poly,y_poly] = p_poly_dist(tcp(1),tcp(2), allx, ally);
            
            set(obstacle.dispalyDis,'Position',[-roomSize*0.8,roomSize*0.8,0],...
                'String',['The minimum distance is: ',num2str(d,'%5.2f')]);
            
            set(obstacle.plotMiniDis,'XData',[tcp(1) x_poly],...
                'YData',[tcp(2),y_poly]);
            
            [allf,allv] = poly2fv(allx/4+3/4*roomSize,ally/4-3/4*roomSize);
            set(obstacle.plotUnkown,'Faces',allf,'Vertices',allv);
        end
    end

    function updateSensorViews(k,obj)
        p = getCrossPoints(sensor(k),obj);
        if (isempty(p))
            return
        end
        p1 = getBoundaryPoints(sensor(k).coord(1:2),p(1:2,1),...
            sensor(k).viewNear,sensor(k).viewFar*2);
        area.shape(1:2,1:2) = [p(1:2,1),p1(1:2,2)];
        p2 = getBoundaryPoints(sensor(k).coord(1:2),p(1:2,2),...
            sensor(k).viewNear,sensor(k).viewFar*2);
        area.shape(1:2,3:4) = [p2(1:2,2),p(1:2,2)];
        d1 = getDistance(p(1:2,1),sensor(k).coord);
        d2 = getDistance(p(1:2,2),sensor(k).coord);
        if (getDistance(p1(1:2,2),sensor(k).coord) < d1 && ...
            getDistance(p2(1:2,2),sensor(k).coord) < d2)
            return;
        end
        if (isempty(sensor(k).viewArea))
            return
        end
        vax2 = sensor(k).viewArea(1,:);
        vay2 = sensor(k).viewArea(2,:);
        tx = vax2;ty = vay2;
        
        if(~ispolycw(vax2,vay2))
            [vax2,vay2] = poly2cw(vax2,vay2);
        end
        
        if (~isempty(obj.shape))
            ox = obj.shape(1,:)+obj.coord(1);
            oy = obj.shape(2,:)+obj.coord(2);
            m = convhull(ox,oy);
            ox = ox(m);oy = oy(m);
            if (~ispolycw(ox,oy))
                [ox,oy] = poly2cw(ox,oy);
            end
            [tx,ty] = polybool('-',vax2,vay2,ox,oy);
        end
        if(~isempty(area.shape))
            ax = area.shape(1,:);
            ay = area.shape(2,:);
            if(~ispolycw(ax,ay))
                [ax,ay] = poly2cw(ax,ay);
            end
            if (~isempty(ax)&& polyarea(ax,ay) > 0.1)
                [tx,ty] = polybool('-',tx,ty,ax,ay);
            end
        end
        
        if(~ispolycw(tx,ty))
            [tx,ty] = poly2cw(tx,ty);
        end
        
        sensor(k).viewArea = [tx;ty];
    end

    function editchange(editIndex)
        switch editIndex
            case 1
                for kk = 1:length(sensor)
                    sensor(kk).viewFar = str2double(editBox(editIndex).String);
                end
            case 2
                for kk = 1:length(sensor)
                    sensor(kk).viewRange = str2double(editBox(editIndex).String);
                end
            case 3
                roomSize = str2double(editBox(editIndex).String);
                roomSize(roomSize<2) = 2;
                for kk = 1:length(walls)
                    walls(kk).shape = walls(kk).initialShape*roomSize/InitialRoomSize;
                    set(walls(kk).plot,'XData',walls(kk).shape(1,:),'YData',walls(kk).shape(2,:))
                end
                set(myAxes,'XLim',[-roomSize*1.33 roomSize*3],'YLim',[-roomSize*1.2 roomSize*1.2])
        end
        myControl.type = 'start';
        myMotionFcn
        myControl.type = '';
    end

    function wallPatchButtonDownFcn
        switch myFig.SelectionType
            case 'normal'
                newLength = length(objs)+1;
                objs(newLength).coord = myAxes.CurrentPoint(1,1:2)';
                objs(newLength).shape = (rand(2,5)*2).*[1 1 -1 -1 1; 1 -1 -1 1 1];
                objs(newLength).shape(:,5) = objs(newLength).shape(:,1);
                objs(newLength).color = [rand*.2 rand*0.2 rand*0.2];
                objs(newLength).plot = patch('XData',objs(newLength).shape(1,:)+objs(newLength).coord(1),...
                                             'YData',objs(newLength).shape(2,:)+objs(newLength).coord(2),...
                                             'FaceColor',objs(newLength).color,'Parent',myAxes,...
                                             'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',newLength));
                % uistack(sensor.plotRay,'top')
        end
        myControl.type = 'start';
        myMotionFcn
        myControl.type = '';
    end
    
    function myScrollWheelFcn(~,evt)
        switch myControl.type
            case 'obj'
                
                if myControl.index < 3
                    if evt.VerticalScrollCount > 0
                        objs(myControl.index).viewDirection = objs(myControl.index).viewDirection -5/180*pi;
                    else
                        objs(myControl.index).viewDirection = objs(myControl.index).viewDirection +5/180*pi;
                    end

                    transform = rotz(-objs(myControl.index).viewDirection*180/pi);
                    transform(1:2,3) = objs(myControl.index).coord(1:2)/1000;
                    objs(myControl.index).shape = transform*objs(myControl.index).originShape;
                else
                    if evt.VerticalScrollCount > 0
                        objs(myControl.index).shape = objs(myControl.index).shape/1.2;
                    else
                        objs(myControl.index).shape = objs(myControl.index).shape*1.2;
                    end
                end
                
                set(objs(myControl.index).plot,'XData',objs(myControl.index).shape(1,:)+objs(myControl.index).coord(1),...
                    'YData',objs(myControl.index).shape(2,:)+objs(myControl.index).coord(2))
                
                myControl.type = 'start';
                myMotionFcn
                myControl.type = 'obj';
            case 'sensor'
                if evt.VerticalScrollCount > 0
                    sensor(myControl.index).viewDirection = sensor(myControl.index).viewDirection - 5/180*pi;
                else
                    sensor(myControl.index).viewDirection = sensor(myControl.index).viewDirection + 5/180*pi;
                end
                sensor(myControl.index).viewDirection = mod(sensor(myControl.index).viewDirection,pi);
                transform = rotz(-sensor(myControl.index).viewDirection*180/pi);
                transform(1:2,3) = sensor(myControl.index).coord(1:2)/1000;  
                origin_shape = [0.5 0.5 -0.5 -0.5;1.5 -1.5 -1 1; 1 1 1 1];
                sensor(myControl.index).shape = transform*origin_shape;
                set(sensor(myControl.index).plotSensor,'XData',sensor(myControl.index).shape(1,:)+sensor(myControl.index).coord(1),...
                                                       'YData',sensor(myControl.index).shape(2,:)+sensor(myControl.index).coord(2));
                myControl.type = 'start';
                myMotionFcn
                myControl.type = 'sensor';
            otherwise
                allRadioButtons = get(scrollWheelButtonGroup,'Children');
                if allRadioButtons(2).Value == 1
                    for kk = 1:length(sensor)
                        sensor(kk).viewRange = sensor(kk).viewRange-evt.VerticalScrollCount/20;
                        sensor(kk).viewRange(sensor(kk).viewRange<0.2) = 0.2;
                        editBox(2).String = num2str(sensor(kk).viewRange);
                    end
                elseif allRadioButtons(3).Value == 1
                    for kk = 1:length(sensor)
                        sensor(kk).viewFar = sensor(kk).viewFar-evt.VerticalScrollCount/5;
                        editBox(1).String = num2str(sensor(kk).viewFar);
                    end
                elseif allRadioButtons(1).Value == 1
                    roomSize = roomSize-evt.VerticalScrollCount;
                    roomSize(roomSize<2) = 2;
                    for kk = 1:length(walls)
                        walls(kk).shape = walls(kk).initialShape*roomSize/InitialRoomSize;
                        set(walls(kk).plot,'XData',walls(kk).shape(1,:),'YData',walls(kk).shape(2,:))
                    end
                    set(myAxes,'XLim',[-roomSize*1.33 roomSize*3],'YLim',[-roomSize*1.2 roomSize*1.2])
                    editBox(3).String = num2str(roomSize);
                end
                myControl.type = 'start';
                myMotionFcn
                myControl.type = '';
        end
    end

    function UIvar = createUIcontrol(varType,varPos,varStr,varFontSize,varFColor,varParent,varVis,varCallback,varBColor)
        UIvar = uicontrol('Style',varType,...
            'units','normalized',...
            'Position',varPos,...
            'String',varStr,...
            'FontSize',varFontSize,...
            'FontName','Arial',...
            'FontUnits','normalized',...
            'BackgroundColor',varBColor,...
            'ForegroundColor',varFColor,...
            'Parent',varParent,...
            'Visible',varVis,...
            'Callback',varCallback,...
            'HorizontalAlignment','center');
    end

    function T = rotz(angle)
        angle = angle/180.0*pi;
        T = [cos(angle) -sin(angle) 0;...
             sin(angle)  cos(angle) 0;...
                     0           0  1];
    end

    function  d = getDistance(p1,p2)
        d = sqrt((p1(1)-p2(1))*(p1(1)-p2(1))+(p1(2)-p2(2))*(p1(2)-p2(2)));
    end

    function p = getBoundaryPoints(p0,p1,r_near,r_far)
        x0 = p0(1); y0 = p0(2);
        x1 = p1(1); y1 = p1(2);

        r = [r_near;r_far];
        p = zeros(2,2);

        if (x1 - x0 ~= 0) 
            a = (y1-y0)/(x1-x0);
            b = a*x1-y1;
            c = b + y0;
            A = a*a + 1;
            B = -(2*a*c+2*x0);
            for i = 1:2
                C = c*c + x0*x0 - r(i)*r(i);
                if B*B - 4*A*C >= 0
                    x = (-B+sqrt(B*B - 4*A*C))/(2*A);
                    y = a*(x-x1) + y1;
                    if [x-x0;y-y0]'*[x1-x0;y1-y0] >= 0
                        p(:,i) = [x;y];
                    else
                        x = (-B-sqrt(B*B - 4*A*C))/(2*A);
                        y = a*(x-x1) + y1;
                        p(:,i) = [x;y];
                    end
                else
                    p(:,i) = [NaN;NaN];
                end
            end
        else
            p(:,1) = [x0,y0+sign(y1-y0)*r(1)];
            p(:,2) = [x0,y0+sign(y1-y0)*r(2)];
        end
    end

    function p = getCrossPoints(sensor,obj)
        % 传感器视场夹角
        vector = rotz(-sensor.viewDirection/pi*180)*[1 0 0]';
        centerVector = vector(1:2);
        centerVector = centerVector/norm(centerVector);

        n = size(obj.shape,2);  % 障碍物顶点数/边数

        leftMax = 0;            rightMax = 0;
        leftMin = pi;           rightMin = pi;
        leftMaxIndex = 0;       rightMaxIndex = 0;
        leftMinIndex = 0;       rightMinIndex = 0;

        for i=1:n
            tempVector = (obj.shape(1:2,i)+ obj.coord - sensor.coord(1:2));
            tempVector = tempVector/norm(tempVector);
            tempAngle = 1 - tempVector'*centerVector;
            ZVector = cross([centerVector;0],[tempVector;0]);
            if ZVector(3) > 0
                if tempAngle < leftMin
                    leftMin = tempAngle;
                    leftMinIndex = i;
                end
                if tempAngle > leftMax
                    leftMax = tempAngle;
                    leftMaxIndex = i;
                end
            else
                if tempAngle < rightMin
                    rightMin = tempAngle;
                    rightMinIndex = i;
                end
                if tempAngle > rightMax
                    rightMax = tempAngle;
                    rightMaxIndex = i;
                end
            end
        end
        leftPoint = []; rightPoint = [];
        if leftMaxIndex ~= 0
            leftPoint = obj.shape(1:2,leftMaxIndex)+ obj.coord;
        else
            if rightMinIndex ~=0
                leftPoint = obj.shape(1:2,rightMinIndex)+ obj.coord;
            end
        end

        if rightMaxIndex ~=0
            rightPoint = obj.shape(1:2,rightMaxIndex)+ obj.coord;
        else
            if leftMinIndex ~=0
                rightPoint = obj.shape(1:2,leftMinIndex)+ obj.coord;
            end
        end

        p = [leftPoint,rightPoint];

    end

    function [d,x_poly,y_poly] = p_poly_dist(x, y, xv, yv) 

        % If (xv,yv) is not closed, close it.
        xv = xv(:);
        yv = yv(:);
        Nv = length(xv);
        if ((xv(1) ~= xv(Nv)) || (yv(1) ~= yv(Nv)))
            xv = [xv ; xv(1)];
            yv = [yv ; yv(1)];
        %     Nv = Nv + 1;
        end

        % linear parameters of segments that connect the vertices
        % Ax + By + C = 0
        A = -diff(yv);
        B =  diff(xv);
        C = yv(2:end).*xv(1:end-1) - xv(2:end).*yv(1:end-1);

        % find the projection of point (x,y) on each rib
        AB = 1./(A.^2 + B.^2);
        vv = (A*x+B*y+C);
        xp = x - (A.*AB).*vv;
        yp = y - (B.*AB).*vv;

        % Test for the case where a polygon rib is 
        % either horizontal or vertical. From Eric Schmitz
        id = find(diff(xv)==0);
        xp(id)=xv(id);
        clear id
        id = find(diff(yv)==0);
        yp(id)=yv(id);

        % find all cases where projected point is inside the segment
        idx_x = (((xp>=xv(1:end-1)) & (xp<=xv(2:end))) | ((xp>=xv(2:end)) & (xp<=xv(1:end-1))));
        idx_y = (((yp>=yv(1:end-1)) & (yp<=yv(2:end))) | ((yp>=yv(2:end)) & (yp<=yv(1:end-1))));
        idx = idx_x & idx_y;

        % distance from point (x,y) to the vertices
        dv = sqrt((xv(1:end-1)-x).^2 + (yv(1:end-1)-y).^2);

        if(~any(idx)) % all projections are outside of polygon ribs
           [d,I] = min(dv);
           x_poly = xv(I);
           y_poly = yv(I);
        else
           % distance from point (x,y) to the projection on ribs
           dp = sqrt((xp(idx)-x).^2 + (yp(idx)-y).^2);
           [min_dv,I1] = min(dv);
           [min_dp,I2] = min(dp);
           [d,I] = min([min_dv min_dp]);
           if I==1 %the closest point is one of the vertices
               x_poly = xv(I1);
               y_poly = yv(I1);
           elseif I==2 %the closest point is one of the projections
               idxs = find(idx);
               x_poly = xp(idxs(I2));
               y_poly = yp(idxs(I2));
           end
        end

        if(inpolygon(x, y, xv, yv)) 
           d = -d;
        end
    end
end