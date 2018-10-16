% % % % % % % % % % % % % % % % % % % % 
%
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
%
% % % % % % % % % % % % % % % % % % % %

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
LB = 0.900; % Length of bucket [m]
WB = 2.340; % Width of bucket [m]

alpha = 1.4; % Velocity of front axle [m/s]
angvellim = 10 *toRad;

%
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

%% Front body
% Front end
X1F(:) = X1(:) + LF * cos( theta1(:)+theta2(:) );
Y1F(:) = Y1(:) + LF * sin( theta1(:)+theta2(:) );

    X1Fr(:) = X1F(:) + W/2 * sin( theta1(:)+theta2(:) );
    Y1Fr(:) = Y1F(:) - W/2 * cos( theta1(:)+theta2(:) );

    X1Fl(:) = X1F(:) - W/2 * sin( theta1(:)+theta2(:) );
    Y1Fl(:) = Y1F(:) + W/2 * cos( theta1(:)+theta2(:) );

% Rear end
X1R(:) = X1(:) - (L-W/2/sqrt(3)) * cos( theta1(:)+theta2(:) );
Y1R(:) = Y1(:) - (L-W/2/sqrt(3)) * sin( theta1(:)+theta2(:) );

    X1Rr(:) = X1R(:) + W/2 * sin( theta1(:)+theta2(:) );
    Y1Rr(:) = Y1R(:) - W/2 * cos( theta1(:)+theta2(:) );

    X1Rl(:) = X1R(:) - W/2 * sin( theta1(:)+theta2(:) );
    Y1Rl(:) = Y1R(:) + W/2 * cos( theta1(:)+theta2(:) );    
    
% Bucket
XBF(:) = X1F(:) + LB * cos( theta1(:)+theta2(:) );
YBF(:) = Y1F(:) + LB * sin( theta1(:)+theta2(:) ); 

    XBFr(:) = XBF(:) + WB/2 * sin( theta1(:)+theta2(:) );
    YBFr(:) = YBF(:) - WB/2 * cos( theta1(:)+theta2(:) );

    XBFl(:) = XBF(:) - WB/2 * sin( theta1(:)+theta2(:) );
    YBFl(:) = YBF(:) + WB/2 * cos( theta1(:)+theta2(:) );

 
    XBRr(:) = X1F(:) + WB/2 * sin( theta1(:)+theta2(:) );
    YBRr(:) = Y1F(:) - WB/2 * cos( theta1(:)+theta2(:) );

    XBRl(:) = X1F(:) - WB/2 * sin( theta1(:)+theta2(:) );
    YBRl(:) = Y1F(:) + WB/2 * cos( theta1(:)+theta2(:) );

% Tire 1 (Right front tire)
XT1_1(:) = X1(:) + (W/2 + Wtire) * sin( theta1(:)+theta2(:) ) ...
    + Dtire/2 * cos( theta1(:)+theta2(:) );
YT1_1(:) = Y1(:) - (W/2 + Wtire) * cos( theta1(:)+theta2(:) ) ...
    + Dtire/2 * sin( theta1(:)+theta2(:) );

XT1_2(:) = X1(:) + W/2 * sin( theta1(:)+theta2(:) ) ...
    + Dtire/2 * cos( theta1(:)+theta2(:) );
YT1_2(:) = Y1(:) - W/2 * cos( theta1(:)+theta2(:) ) ...
    + Dtire/2 * sin( theta1(:)+theta2(:) );
    
XT1_3(:) = X1(:) + W/2 * sin( theta1(:)+theta2(:) ) ...
    - Dtire/2 * cos( theta1(:)+theta2(:) );
YT1_3(:) = Y1(:) - W/2 * cos( theta1(:)+theta2(:) ) ...
    - Dtire/2 * sin( theta1(:)+theta2(:) );
    
XT1_4(:) = X1(:) + (W/2 + Wtire) * sin( theta1(:)+theta2(:) ) ...
    - Dtire/2 * cos( theta1(:)+theta2(:) );
YT1_4(:) = Y1(:) - (W/2 + Wtire) * cos( theta1(:)+theta2(:) ) ...
    - Dtire/2 * sin( theta1(:)+theta2(:) );
        
% Tire 2 (Left front tire)
XT2_1(:) = X1(:) - (W/2 + Wtire) * sin( theta1(:)+theta2(:) ) ...
    + Dtire/2 * cos( theta1(:)+theta2(:) );
YT2_1(:) = Y1(:) + (W/2 + Wtire) * cos( theta1(:)+theta2(:) ) ...
    + Dtire/2 * sin( theta1(:)+theta2(:) );

XT2_2(:) = X1(:) - W/2 * sin( theta1(:)+theta2(:) ) ...
    + Dtire/2 * cos( theta1(:)+theta2(:) );
YT2_2(:) = Y1(:) + W/2 * cos( theta1(:)+theta2(:) ) ...
    + Dtire/2 * sin( theta1(:)+theta2(:) );
    
XT2_3(:) = X1(:) - W/2 * sin( theta1(:)+theta2(:) ) ...
    - Dtire/2 * cos( theta1(:)+theta2(:) );
YT2_3(:) = Y1(:) + W/2 * cos( theta1(:)+theta2(:) ) ...
    - Dtire/2 * sin( theta1(:)+theta2(:) );
    
XT2_4(:) = X1(:) - (W/2 + Wtire) * sin( theta1(:)+theta2(:) ) ...
    - Dtire/2 * cos( theta1(:)+theta2(:) );
YT2_4(:) = Y1(:) + (W/2 + Wtire) * cos( theta1(:)+theta2(:) ) ...
    - Dtire/2 * sin( theta1(:)+theta2(:) );


%% Rear body
% Front end
X2F(:) = X2(:) + (L-W/2/sqrt(3)) * cos( theta2(:) );
Y2F(:) = Y2(:) + (L-W/2/sqrt(3)) * sin( theta2(:) );

    X2Fr(:) = X2F(:) + W/2 * sin( theta2(:) );
    Y2Fr(:) = Y2F(:) - W/2 * cos( theta2(:) );

    X2Fl(:) = X2F(:) - W/2 * sin( theta2(:) );
    Y2Fl(:) = Y2F(:) + W/2 * cos( theta2(:) );

% Rear end
X2R(:) = X2(:) - LR * cos( theta2(:) );
Y2R(:) = Y2(:) - LR * sin( theta2(:) );

    X2Rr(:) = X2R(:) + W/2 * sin( theta2(:) );
    Y2Rr(:) = Y2R(:) - W/2 * cos( theta2(:) );

    X2Rl(:) = X2R(:) - W/2 * sin( theta2(:) );
    Y2Rl(:) = Y2R(:) + W/2 * cos( theta2(:) );    
        
% Tire 3 (Right rear tire)
XT3_1(:) = X2(:) - (W/2 + Wtire) * sin( theta2(:) ) + Dtire/2 * cos( theta2(:) );
YT3_1(:) = Y2(:) + (W/2 + Wtire) * cos( theta2(:) ) + Dtire/2 * sin( theta2(:) );

XT3_2(:) = X2(:) - W/2 * sin( theta2(:) ) + Dtire/2 * cos( theta2(:) );
YT3_2(:) = Y2(:) + W/2 * cos( theta2(:) ) + Dtire/2 * sin( theta2(:) );
    
XT3_3(:) = X2(:) - W/2 * sin( theta2(:) ) - Dtire/2 * cos( theta2(:) );
YT3_3(:) = Y2(:) + W/2 * cos( theta2(:) ) - Dtire/2 * sin( theta2(:) );
    
XT3_4(:) = X2(:) - (W/2 + Wtire) * sin( theta2(:) ) - Dtire/2 * cos( theta2(:) );
YT3_4(:) = Y2(:) + (W/2 + Wtire) * cos( theta2(:) ) - Dtire/2 * sin( theta2(:) );

% tire 4 (Left rear tire)
XT4_1(:) = X2(:) + (W/2 + Wtire) * sin( theta2(:) ) + Dtire/2 * cos( theta2(:) );
YT4_1(:) = Y2(:) - (W/2 + Wtire) * cos( theta2(:) ) + Dtire/2 * sin( theta2(:) );

XT4_2(:) = X2(:) + W/2 * sin( theta2(:) ) + Dtire/2 * cos( theta2(:) );
YT4_2(:) = Y2(:) - W/2 * cos( theta2(:) ) + Dtire/2 * sin( theta2(:) );
    
XT4_3(:) = X2(:) + W/2 * sin( theta2(:) ) - Dtire/2 * cos( theta2(:) );
YT4_3(:) = Y2(:) - W/2 * cos( theta2(:) ) - Dtire/2 * sin( theta2(:) );
    
XT4_4(:) = X2(:) + (W/2 + Wtire) * sin( theta2(:) ) - Dtire/2 * cos( theta2(:) );
YT4_4(:) = Y2(:) - (W/2 + Wtire) * cos( theta2(:) ) - Dtire/2 * sin( theta2(:) );


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

FrontBody = patch([X1Fr(q) X1Fl(q) X1Rl(q) X3(q) X1Rr(q)], ...
    [Y1Fr(q) Y1Fl(q) Y1Rl(q) Y3(q) Y1Rr(q)],[240 142 44]/255,'LineWidth',2);

Bucket = patch([XBFr(q) XBFl(q)  XBRl(q) XBRr(q)], ...
    [YBFr(q) YBFl(q)  YBRl(q) YBRr(q)],[240 142 44]/255,'LineWidth',2);

Tire1 = patch([XT1_1(q) XT1_2(q) XT1_3(q) XT1_4(q)], ...
    [YT1_1(q) YT1_2(q) YT1_3(q) YT1_4(q)],'k');

Tire2 = patch([XT2_1(q) XT2_2(q) XT2_3(q) XT2_4(q)], ...
    [YT2_1(q) YT2_2(q) YT2_3(q) YT2_4(q)],'k');

RearBody = patch([X2Fr(q) X3(q) X2Fl(q) X2Rl(q) X2Rr(q)], ...
    [Y2Fr(q) Y3(q) Y2Fl(q) Y2Rl(q) Y2Rr(q)],[240 142 44]/255,'LineWidth',2);

Tire3 = patch([XT3_1(q) XT3_2(q) XT3_3(q) XT3_4(q)], ...
    [YT3_1(q) YT3_2(q) YT3_3(q) YT3_4(q)],'k');

Tire4 = patch([XT4_1(q) XT4_2(q) XT4_3(q) XT4_4(q)], ...
    [YT4_1(q) YT4_2(q) YT4_3(q) YT4_4(q)],'k');

Articulation = plot(X3(q),Y3(q),'ko','MarkerFaceColor','r');

Tdisp = floor(time);
TimeDisp =  text('units','pixels','position',[20 20 0], ...
    'fontsize',20,'string',strcat(num2str(Tdisp(1)),' s'));

% Update graphic
count = 10;
for q = 1:count:length(time)
    set(FrontBody,'Xdata',[X1Fr(q) X1Fl(q) X1Rl(q) X3(q) X1Rr(q)], ...
        'Ydata',[Y1Fr(q) Y1Fl(q) Y1Rl(q) Y3(q) Y1Rr(q)])
    set(Bucket,'Xdata',[XBFr(q) XBFl(q)  XBRl(q) XBRr(q)], ...
        'Ydata',[YBFr(q) YBFl(q)  YBRl(q) YBRr(q)])
    set(Tire1,'Xdata',[XT1_1(q) XT1_2(q) XT1_3(q) XT1_4(q)], ...
        'Ydata',[YT1_1(q) YT1_2(q) YT1_3(q) YT1_4(q)])
    set(Tire2,'Xdata',[XT2_1(q) XT2_2(q) XT2_3(q) XT2_4(q)], ...
        'Ydata',[YT2_1(q) YT2_2(q) YT2_3(q) YT2_4(q)])
    set(RearBody,'Xdata',[X2Fr(q) X3(q) X2Fl(q) X2Rl(q) X2Rr(q)], ...
        'Ydata',[Y2Fr(q) Y3(q) Y2Fl(q) Y2Rl(q) Y2Rr(q)])
    set(Tire3,'Xdata',[XT3_1(q) XT3_2(q) XT3_3(q) XT3_4(q)], ...
        'Ydata',[YT3_1(q) YT3_2(q) YT3_3(q) YT3_4(q)])
    set(Tire4,'Xdata',[XT4_1(q) XT4_2(q) XT4_3(q) XT4_4(q)], ...
        'Ydata',[YT4_1(q) YT4_2(q) YT4_3(q) YT4_4(q)])
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