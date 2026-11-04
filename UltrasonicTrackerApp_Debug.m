function UltrasonicTrackerApp_Debug()
    clc; close all;

    % --- UI ---
    fig = uifigure('Name','Ultrasonic 3D Tracker (Debug)','Position',[100 100 980 540]);

    ax = uiaxes(fig,'Position',[30 90 540 420]); 
    grid(ax,'on'); hold(ax,'on');
    xlabel(ax,'X (cm)'); ylabel(ax,'Y (cm)'); zlabel(ax,'Z (cm)');
    xlim(ax,[-25 25]); ylim(ax,[-40 40]); zlim(ax,[0 200]); 
    view(ax,[45 30]);
    title(ax,'Real-time 3D Position (cm)');
    hPoint = plot3(ax,0,0,0,'go','MarkerSize',8,'MarkerFaceColor','g');
    hLabel = text(ax,0,0,0,'','FontSize',12,'Color','b','FontWeight','bold');

    infoPanel = uipanel(fig,'Title','ورودی‌ها و خروجی‌ها','FontWeight','bold', ...
        'Position',[590 90 360 420]);

    lblD = gobjects(1,5);
    for i = 1:5
        lblD(i) = uilabel(infoPanel,'Text',sprintf('d%d: -- cm',i), ...
            'Position',[10 380-(i-1)*25 200 22],'FontSize',12);
    end
    lblLaser = uilabel(infoPanel,'Text','Laser Height: -- cm', ...
        'Position',[10 255 200 22],'FontSize',12);

    lblX = uilabel(infoPanel,'Text','X: -- cm','Position',[10 135 200 22], ...
        'FontSize',14,'FontWeight','bold');
    lblY = uilabel(infoPanel,'Text','Y: -- cm','Position',[10 110 200 22], ...
        'FontSize',14,'FontWeight','bold');
    lblZ = uilabel(infoPanel,'Text','Z: -- cm','Position',[10 85 200 22], ...
        'FontSize',14,'FontWeight','bold');
    lblStatus = uilabel(infoPanel,'Text','Status: Waiting...','Position',[10 55 330 22], ...
        'FontSize',12,'FontColor',[.2 .2 .2]);

    rawPanel = uipanel(fig,'Title','آخرین خط دریافتی','FontWeight','bold', ...
        'Position',[30 20 920 60]);
    lblRaw = uilabel(rawPanel,'Text','(empty)','Position',[10 5 900 22], ...
        'FontName','Consolas','FontSize',11);

    % --- Serial (event-driven) ---
    port = "COM6";           % پورت صحیح
    baud = 9600;             % با آردوینو هماهنگ کن
    s = serialport(port, baud, "Timeout", 1);
    configureTerminator(s, "CR/LF");  
    pause(0.2); flush(s);

    % ذخیره هندل‌ها
    s.UserData.lbls = struct('D',{lblD}, 'Laser',lblLaser, ...
                             'X',lblX,'Y',lblY,'Z',lblZ, ...
                             'Raw',lblRaw,'Status',lblStatus);
    s.UserData.plot = struct('ax',ax,'pt',hPoint,'lab',hLabel);

    % کال‌بک روی خط کامل
    configureCallback(s, "terminator", @(src,evt) onSerialLine(src));

    % بستن سالم
    fig.CloseRequestFcn = @(~,~) onClose(fig, s);

    function onSerialLine(src)
        try
            line = char(readline(src));
            line = strrep(line, char(13), ''); % حذف CR
            line = strtrim(line);
            if isempty(line), return; end

            U = src.UserData;
            U.lbls.Raw.Text = line;

            parts = split(line, ',');
            nums = str2double(parts);
            if numel(nums) < 6 || any(isnan(nums(1:6)))
                U.lbls.Status.Text = sprintf('Status: Skipped (count=%d/NaN)', numel(nums));
                return;
            end

            % map
            d = nums(1:5);            % d1..d5
            laserHeight = nums(6);    % cm

            % نمایش ورودی‌ها
            for i=1:5
                U.lbls.D(i).Text = sprintf('d%d: %.1f cm', i, d(i));
            end
            U.lbls.Laser.Text = sprintf('Laser Height: %.1f cm', laserHeight);

            % هندسه XY
            d5 = 315/2; % فاصله ثابت بین سنسور مرجع و مرکز
            d1 = d(2); d2 = d(3); d3 = d(4); d4 = d(5);
            L  = d5 + laserHeight;

            dxy = [ ...
                sqrt(max((d1 - d5)^2 - L^2, 0));
                sqrt(max((d2 - d5)^2 - L^2, 0));
                sqrt(max((d3 - d5)^2 - L^2, 0));
                sqrt(max((d4 - d5)^2 - L^2, 0)) ];

            [xMeas, yMeas] = trilat_ls_resilient(dxy);
            zMeas = laserHeight;

            % استفاده مستقیم از اندازه‌گیری‌ها
            X = xMeas; Y = yMeas; Z = zMeas;

            % به‌روزرسانی گرافیک
            set(U.plot.pt,'XData',X,'YData',Y,'ZData',Z);
            set(U.plot.lab,'Position',[X,Y,Z+5], ...
                'String',sprintf('X=%.1f cm\nY=%.1f cm\nZ=%.1f cm',X,Y,Z));
            U.lbls.X.Text = sprintf('X: %.1f cm', X);
            U.lbls.Y.Text = sprintf('Y: %.1f cm', Y);
            U.lbls.Z.Text = sprintf('Z: %.1f cm', Z);

            U.lbls.Status.Text = 'Status: Updated ✓';
            drawnow limitrate nocallbacks;

            src.UserData = U; % ذخیره تغییرات

        catch ME
            try
                src.UserData.lbls.Status.Text = ['Error: ' ME.message];
            end
        end
    end
end

% --- تری‌لترِیشن مقاوم ---
function [x, y] = trilat_ls_resilient(dxy)
    S = [ -37, 20;
           37, 20;
          -37,-20;
           37,-20 ];
    valid = isfinite(dxy) & dxy > 0;
    idx = find(valid);
    if numel(idx) < 2
        x = 0; y = 0; return
    end
    s1 = S(idx(1),:);
    d1 = dxy(idx(1));
    if numel(idx) == 2
        x = s1(1); y = s1(2); return
    end
    A = 2*(S(idx(2:end),:) - s1);
    b = d1^2 - dxy(idx(2:end)).^2 + sum(S(idx(2:end),:).^2,2) - sum(s1.^2);
    pos = A\b;
    x = pos(1); y = pos(2);
end

% --- بستن سالم ---
function onClose(fig, s)
    try, configureCallback(s, "off"); catch, end
    try, flush(s); catch, end
    try, clear s; catch, end
    delete(fig);
end
