% % % % % % % % % % % % % % % % % % % % % % % % %
% LoaderPathTracking.m
%
% Description: https://mechlog.hateblo.jp/entry/2018/10/15/132758
%
% Environment: Matlab 2016b with Control System Toolbox
%
% Author: twitter @mech_log
%
% Refrence:
% Shiroma, N. et al., "Nonlinear StraightPath Tracking Control
% for an Articulated Steering Type Vehicle",
% Transactions of the Japan Society of Mechanical Engineers,
% Series(C), Vol.76, No.763, 2010, pp.151?158. (in Japanese)
% https://www.jstage.jst.go.jp/article/kikaic1979/76/763/76_KJ00006168003/_article/-char/ja/
% % % % % % % % % % % % % % % % % % % % % % % % %

clear; close all;

toRad = pi/180;
st = 0.01;  % Sampling time [sec]
endtime = 20; % [sec]
time = 0:st:endtime;

L  = 1.300; % 1/2 of wheelbase [m]
LF = 1.005; % Front overhang [m]
LR = 1.680; % Rear overhang [m]
W  = 1.310; % Width [m]
Dtire = 1.044; % Diameter of tire [m]
Wtire = 0.470; % Width of tire [m]
Lbucket = 0.900; % Length of bucket [m]
Wbucket = 2.340; % Width of bucket [m]

alpha = 1.4; % Velocity of front axle [m/s]
angvellim = 10 *toRad;

X1 = zeros(1,length(time));
Y1 = zeros(1,length(time));
theta1 = zeros(1,length(time));
theta2 = zeros(1,length(time));


%% Initial conditions
theta1(1) = 0 *toRad;
theta2(1) = 0 *toRad;
X1(1) = 0; % [m]
Y1(1) = 5; % [m]


%% Vehicle model and controller
Ac = [0 1 0;
    0 0 1;
    0 0 0];

Bc = [0; 0; 1];

p = [-0.5 -0.5 -0.5];

F = acker(Ac,Bc,p);

for q = 1:length(time)-1
    
    xi = [Y1(q);
        alpha * tan(theta1(q)+theta2(q));
        alpha^2 * tan(theta1(q)/2) / (L * (cos(theta1(q)+theta2(q)))^3)];
    
    nu = - F * xi;
    
    upre = -6*alpha * (sin(theta1(q)/2))^2 * sin(theta1(q)+theta2(q)) ...
        / (L * (cos(theta1(q)+theta2(q)))^2) ...
        + 2*L * (cos(theta1(q)+theta2(q)))^3 ...
        * (cos(theta1(q)/2))^2 / alpha^2 * nu;
    
    u = u_limiter(upre,angvellim);
    
    theta1(q+1) = integ(theta1(q), u, st);
    theta2(q+1) = integ(theta2(q), alpha * tan(theta1(q)/2) ...
        / (L * cos(theta1(q)+theta2(q))) - u, st);
    
    X1(q+1) = integ(X1(q), alpha, st);
    Y1(q+1) = integ(Y1(q), alpha * tan(theta1(q)+theta2(q)), st);
end


%% Vehicle positions
% Articulation point
X3(:) = X1(:) - L * cos( theta1(:)+theta2(:) );
Y3(:) = Y1(:) - L * sin( theta1(:)+theta2(:) );

% Center point of rear axle
X2(:) = X3(:) - L * cos( theta2(:) );
Y2(:) = Y3(:) - L * sin( theta2(:) );

% Front body
[X1F, Y1F, X1R, Y1R, XBF, YBF, XBR, YBR, Xtire1, Ytire1, Xtire2, Ytire2] ...
    = FrontBodyEdge(X1(:), Y1(:), theta1(:), theta2(:), L, LF, W, Dtire, Wtire, Lbucket, Wbucket);

% Rear body
[X2F, Y2F, X2R, Y2R, Xtire3, Ytire3, Xtire4, Ytire4] ...
    = RearBodyEdge(X2(:), Y2(:), theta2(:), L, LR, W, Dtire, Wtire);


%% Animation
figure(1)
hold on; axis equal; box on; grid on;
xlim([-10 30])
ylim([-5 10])
xlabel('Displacement \itX\rm [m]')
ylabel('Displacement \itY\rm [m]')

width = 800;
height = width;
pos = get(gcf,'Position');
pos(3)=width; pos(4)=height;
set(gcf,'Position',pos);

q = 1;

plot([-10 30], [0 0],'r-','LineWidth',2); % Reference path

FrontBody = patch([X1F(q,1) X1F(q,2) X1R(q,2) X3(q) X1R(q,1)], ...
    [Y1F(q,1) Y1F(q,2) Y1R(q,2) Y3(q) Y1R(q,1)],[240 142 44]/255,'LineWidth',2);

Bucket = patch([XBF(q,1) XBF(q,2)  XBR(q,2) XBR(q,1)], ...
    [YBF(q,1) YBF(q,2)  YBR(q,2) YBR(q,1)],[240 142 44]/255,'LineWidth',2);

Tire1 = patch([Xtire1(q,1) Xtire1(q,2) Xtire1(q,3) Xtire1(q,4)], ...
    [Ytire1(q,1) Ytire1(q,2) Ytire1(q,3) Ytire1(q,4)],'k');

Tire2 = patch([Xtire2(q,1) Xtire2(q,2) Xtire2(q,3) Xtire2(q,4)], ...
    [Ytire2(q,1) Ytire2(q,2) Ytire2(q,3) Ytire2(q,4)],'k');

RearBody = patch([X2F(q,1) X3(q) X2F(q,2) X2R(q,2) X2R(q,1)], ...
    [Y2F(q,1) Y3(q) Y2F(q,2) Y2R(q,2) Y2R(q,1)],[240 142 44]/255,'LineWidth',2);

Tire3 = patch([Xtire3(q,1) Xtire3(q,2) Xtire3(q,3) Xtire3(q,4)], ...
    [Ytire3(q,1) Ytire3(q,2) Ytire3(q,3) Ytire3(q,4)],'k');

Tire4 = patch([Xtire4(q,1) Xtire4(q,2) Xtire4(q,3) Xtire4(q,4)], ...
    [Ytire4(q,1) Ytire4(q,2) Ytire4(q,3) Ytire4(q,4)],'k');

Articulation = plot(X3(q),Y3(q),'ko','MarkerFaceColor','r');

Tdisp = floor(time);
TimeDisp =  text('units','pixels','position',[20 20 0], ...
    'fontsize',20,'string',strcat(num2str(Tdisp(1)),' s'));

% Update graphic
count = 10;
for q = 1:count:length(time)
    set(FrontBody,'Xdata',[X1F(q,1) X1F(q,2) X1R(q,2) X3(q) X1R(q,1)], ...
        'Ydata',[Y1F(q,1) Y1F(q,2) Y1R(q,2) Y3(q) Y1R(q,1)])
    set(Bucket,'Xdata',[XBF(q,1) XBF(q,2)  XBR(q,2) XBR(q,1)], ...
        'Ydata',[YBF(q,1) YBF(q,2)  YBR(q,2) YBR(q,1)])
    set(Tire1,'Xdata',[Xtire1(q,1) Xtire1(q,2) Xtire1(q,3) Xtire1(q,4)], ...
        'Ydata',[Ytire1(q,1) Ytire1(q,2) Ytire1(q,3) Ytire1(q,4)])
    set(Tire2,'Xdata',[Xtire2(q,1) Xtire2(q,2) Xtire2(q,3) Xtire2(q,4)], ...
        'Ydata',[Ytire2(q,1) Ytire2(q,2) Ytire2(q,3) Ytire2(q,4)])
    set(RearBody,'Xdata',[X2F(q,1) X3(q) X2F(q,2) X2R(q,2) X2R(q,1)], ...
        'Ydata',[Y2F(q,1) Y3(q) Y2F(q,2) Y2R(q,2) Y2R(q,1)])
    set(Tire3,'Xdata',[Xtire3(q,1) Xtire3(q,2) Xtire3(q,3) Xtire3(q,4)], ...
        'Ydata',[Ytire3(q,1) Ytire3(q,2) Ytire3(q,3) Ytire3(q,4)])
    set(Tire4,'Xdata',[Xtire4(q,1) Xtire4(q,2) Xtire4(q,3) Xtire4(q,4)], ...
        'Ydata',[Ytire4(q,1) Ytire4(q,2) Ytire4(q,3) Ytire4(q,4)])
    set(Articulation,'Xdata',X3(q),'Ydata', Y3(q))
    
    set(TimeDisp,'visible','off')
    TimeDisp =  text('units','pixels','position',[20 20 0], ...
        'fontsize',20,'string',strcat(num2str(Tdisp(q)),' s'));
    Frame(q) = getframe;
    drawnow
end


%% Save animation
Frame = Frame(1:count:length(time));
LoaderAnimation = VideoWriter('LoaderAnimation');
open(LoaderAnimation)
writeVideo(LoaderAnimation,Frame)
close(LoaderAnimation)


%% % % % %
function y = integ( x, xdot, dt )
y = x + dt * xdot;
end

function uOut = u_limiter( u, angvellim )
if(u > angvellim)
    uOut = angvellim;
elseif(u < -angvellim)
    uOut = -angvellim;
else
    uOut = u;
end
end

function [X1F, Y1F, X1R, Y1R, XBF, YBF, XBR, YBR, Xtire1, Ytire1, Xtire2, Ytire2] ...
    = FrontBodyEdge(X1, Y1, theta1, theta2, L, LF, W, Dtire, Wtire, Lbucket, Wbucket)
% Front end
X1Fcenter(:) = X1(:) + LF * cos( theta1(:)+theta2(:) );
Y1Fcenter(:) = Y1(:) + LF * sin( theta1(:)+theta2(:) );
[X1F, Y1F] = EndLinePlot(X1Fcenter, Y1Fcenter, theta1(:)+theta2(:), W);

% Rear end
X1Rcenter(:) = X1(:) - (L-W/2/sqrt(3)) * cos( theta1(:)+theta2(:) );
Y1Rcenter(:) = Y1(:) - (L-W/2/sqrt(3)) * sin( theta1(:)+theta2(:) );
[X1R, Y1R] = EndLinePlot(X1Rcenter, Y1Rcenter, theta1(:)+theta2(:), W);

% Bucket
XBFcenter(:) = X1Fcenter(:) + Lbucket * cos( theta1(:)+theta2(:) );
YBFcenter(:) = Y1Fcenter(:) + Lbucket * sin( theta1(:)+theta2(:) );
[XBF, YBF] = EndLinePlot(XBFcenter, YBFcenter, theta1(:)+theta2(:), Wbucket);
[XBR, YBR] = EndLinePlot(X1Fcenter, Y1Fcenter, theta1(:)+theta2(:), Wbucket);

% Front tire
[XfrontTireCenter, YfrontTireCenter] = EndLinePlot(X1(:), Y1(:), theta1(:)+theta2(:), W+Wtire);
% Right front tire
[Xtire1, Ytire1] = TirePlot(XfrontTireCenter(:,1), YfrontTireCenter(:,1), theta1(:)+theta2(:), Dtire, Wtire);
% Left front tire
[Xtire2, Ytire2] = TirePlot(XfrontTireCenter(:,2), YfrontTireCenter(:,2), theta1(:)+theta2(:), Dtire, Wtire);
end

function [X2F, Y2F, X2R, Y2R, Xtire3, Ytire3, Xtire4, Ytire4] ...
    = RearBodyEdge(X2, Y2, theta2, L, LR, W, Dtire, Wtire)
% Front end
X2Fcenter(:) = X2(:) + (L-W/2/sqrt(3)) * cos( theta2(:) );
Y2Fcenter(:) = Y2(:) + (L-W/2/sqrt(3)) * sin( theta2(:) );
[X2F, Y2F] = EndLinePlot(X2Fcenter, Y2Fcenter, theta2(:), W);

% Rear end
X2Rcenter(:) = X2(:) - LR * cos( theta2(:) );
Y2Rcenter(:) = Y2(:) - LR * sin( theta2(:) );
[X2R, Y2R] = EndLinePlot(X2Rcenter, Y2Rcenter, theta2(:), W);

% Rear tire
[XrearTireCenter, YrearTireCenter] = EndLinePlot(X2(:), Y2(:), theta2(:), W+Wtire);
% Right rear tire
[Xtire3, Ytire3] = TirePlot(XrearTireCenter(:,2), YrearTireCenter(:,2), theta2(:), Dtire, Wtire);
% Left rear tire
[Xtire4, Ytire4] = TirePlot(XrearTireCenter(:,1), YrearTireCenter(:,1), theta2(:), Dtire, Wtire);
end

function [X, Y] = EndLinePlot(Xcenter, Ycenter, theta, W)
X(:,1) = Xcenter(:) + W/2 * sin( theta(:) );
Y(:,1) = Ycenter(:) - W/2 * cos( theta(:) );

X(:,2) = Xcenter(:) - W/2 * sin( theta(:) );
Y(:,2) = Ycenter(:) + W/2 * cos( theta(:) );
end

function [Xtire, Ytire] = TirePlot(XtireCenter, YtireCenter, theta, Dtire, Wtire)
Xtire(:,1) = XtireCenter(:) + Wtire/2 * sin(theta(:)) + Dtire/2 * cos(theta(:));
Ytire(:,1) = YtireCenter(:) - Wtire/2 * cos(theta(:)) + Dtire/2 * sin(theta(:));

Xtire(:,2) = XtireCenter(:) - Wtire/2 * sin(theta(:)) + Dtire/2 * cos(theta(:));
Ytire(:,2) = YtireCenter(:) + Wtire/2 * cos(theta(:)) + Dtire/2 * sin(theta(:));

Xtire(:,3) = XtireCenter(:) - Wtire/2 * sin(theta(:)) - Dtire/2 * cos(theta(:));
Ytire(:,3) = YtireCenter(:) + Wtire/2 * cos(theta(:)) - Dtire/2 * sin(theta(:));

Xtire(:,4) = XtireCenter(:) + Wtire/2 * sin(theta(:)) - Dtire/2 * cos(theta(:));
Ytire(:,4) = YtireCenter(:) - Wtire/2 * cos(theta(:)) - Dtire/2 * sin(theta(:));
end