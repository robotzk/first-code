% original paper: Arbitrary Configuration Stabilization Control for Nonholonomic Vehicle With Input Saturation: A c-Nonholonomic Trajectory Approach

% Authors: zk
% Created:  2022 10 29
% Updated:  2022 10 30
clear
clc
close all
dt = 0.5;
zk = figure;
axis equal
axis([-15 15 -15 15]);
grid on
x0 = -5;y0 = 5;yaw = pi/2;%inital contidon
[p,h] = plot_uav(x0, y0, yaw);
hold on
disp('单击鼠标左键点两次设定目标点位置和方向角');
disp('单击鼠标右键点结束');
but = 1;x = x0; y = y0;
p1 = plot(0, 0,'--');
p2 = plot(0, 0, 'linewidth',2,'color','r');
p3 = plot(0, 0,'--');
p4 = plot(0, 0,'--');
p5 = plot(0, 0,'--');
p6 = plot(0, 0,'--');
while but == 1
    
[x_,y_,but]=ginput(1);
if but~=1
    break
end
[xt,yt,but]=ginput(1);
yaw_ = atan2(yt - y_, xt - x_);
delete(p3);
p3 = plot([x_ xt],[y_ yt], 'linewidth',1,'color','b');
delete(p4);
p4 = scatter(x_,y_,100,'p','filled','g');
delete(p5);
p5 = arrow22([xt yt],[xt - x_  yt - y_],1,pi/8,20,'b','k');

if but~=1
    break
end
delete(p1);

gd = SE_2(x_, y_, yaw_);
g_  = SE_2(x, y, yaw);
g = gd \ g_;
xe = g(1, 3);
ye = g(2, 3);

r = 0.5*(xe*xe + ye*ye) / ye;
xc = [x_;y_] + rot_2d(yaw_) * [0;r];
theta = linspace(0,2*pi);
x_cir = xc(1) + abs(r).*cos(theta);
y_cir = xc(2) + abs(r).*sin(theta);

p1 = plot(x_cir, y_cir,'color','b','linewidth',2,'linestyle','--');

X0 = [x;y;yaw;x_;y_;yaw_];
clear X;
X = X0;
j = 1;
while norm(X(1:2, j) - X(4:5, j)) > 0.1 && abs(X(3, j) - X(6, j)) > 0.01
    
    X_ = RK(@model, X(:, j), dt);
    
    j = j + 1;
    X(:, j) = X_;
end
t = cumsum(ones(j, 1) * dt);
% for i = 1:length(X)
%     dX(:,i) = model(0,X(i,:));
% end
H = legend([p,p2,p1,p3,p4],'agent','trajectory','reference','head','goal');
for i = 1:j
    
    delete(p)
    delete(h)
    delete(p2)
    delete(p6)
    [p,h] = plot_uav(X(1, i), X(2, i), X(3, i));
    p2 = plot(X(1, 1:i), X(2, 1:i), 'linewidth',2,'color','r');
    H = legend([p,p2,p1,p3,p4],'agent','trajectory','reference','head','goal');
        set(H,...
        'Position',[0.247916666666667 0.738582677165354 0.150694444444444 0.101574803149606],...
        'FontSize',8,...
        'FontName','Times New Roman');
        p6 = annotation(zk,'textbox',...
        [0.45054128440367 0.160246533127889 0.138908256880734 0.0724191063174126],...
        'String',{['Time=' num2str(t(i)) 's']},...
        'LineStyle','none',...
        'FitBoxToText','off','FontName','Times New Roman');
    pause(0.01)
%     axis equal
end

x = X(1, i);
y = X(2, i);
yaw = X(3, i);
if norm([x y] - [x_ y_]) > 1
    disp('sim-time is too less');
end
end
function A = rot_2d(o)
A = [cos(o)  -sin(o)
    sin(o)    cos(o)];
end
function h1=arrow22(x0,dx0,L,Angle,N,C1,C2)

if nargin<=6;C2='none';end
if nargin<=5;C1='y';end
if nargin<=4;N=2;end

cc=norm(dx0);
if cc<eps; cc=cc+eps;end
s2=dx0./cc;s1=[s2(2),-s2(1)];

t=linspace(-L,L,3);
x=x0(1)+t.*tan(Angle).*s1(1)-abs(t).*s2(1);
y=x0(2)+t.*tan(Angle).*s1(2)-abs(t).*s2(2);

h=L+L*(tan(Angle))^2;r=h*sin(Angle);
t1=linspace(Angle,pi-Angle,N);
x1=x0(1)-h*s2(1)+r*cos(t1).*s1(1)+r*sin(t1).*s2(1);
y1=x0(2)-h*s2(2)+r*cos(t1).*s1(2)+r*sin(t1).*s2(2); 

h1=fill([x x1],[y y1],C1,'edgecolor',C2);
end
function dX = model(t, X)

% gd = SE_2(2, 1, 0);
% gd = SE_2(-1, 2, pi/6);
% gd = SE_2(-2, -1, -pi/6);
gd = SE_2(X(4), X(5), X(6));

g_  = SE_2(X(1), X(2), X(3));
g = gd \ g_;
xe = g(1, 3);
ye = g(2, 3);
oe = asin(g(2,1));

if g(1, 1) < 0
    oe = sign(oe)*pi - oe;
end
if oe == -pi
    oe = pi;
end
alpha = 2* atan(ye/xe);


ae = oe - alpha;

k1 = 0.2;
k2 = 0.2;
v = -k1 * tanh(xe * cos(oe) + ye * sin(oe));
w = -k2 * tanh(ae) + 2*v* (xe*sin(oe) - ye*cos(oe)) / (xe * xe + ye * ye);

dX = zeros(6,1);
dX(1) = v * cos(X(3));
dX(2) = v * sin(X(3));
dX(3) = w;




function G = SE_2(x, y, theta)
G = zeros(3 , 3);
G(1,1) = cos(theta);
G(1,2) = -sin(theta);
G(2,2) = cos(theta);
G(2,1) = sin(theta);
G(1,3) = x;
G(2,3) = y;
G(3,3) = 1;
end
end
function y_ = RK(fun, y, dt)
t = 0;
k1 = fun(t,y)*dt;
k2 = fun(t,y + 0.5*k1)*dt;
k3 = fun(t,y + 0.5*k2)*dt;
k4 = fun(t,y + k3)*dt;
y_ = y + (k1 + 2*k2 + 2*k3 + k4)/6;
end

function G = SE_2(x, y, theta)
G = zeros(3 , 3);
G(1,1) = cos(theta);
G(1,2) = -sin(theta);
G(2,2) = cos(theta);
G(2,1) = sin(theta);
G(1,3) = x;
G(2,3) = y;
G(3,3) = 1;
end
function [p,h] = plot_uav(x, y, yaw, color)

% example: plot_uav(0, 0, 0, 'g')
if nargin<=3
    color='y';
end
R = 0.1;
theta = linspace(0, 2*pi);
X = R * cos(theta);
Y = R * sin(theta);
X = X + x;
Y = Y + y;

%uav_size
 a = [0 0
     4 0
     6 1
     4 2
     0 2
     ];
 a = a - [3 1];
 a = a./4;
xc = [x, y];
A = [cos(yaw)  -sin(yaw)
    sin(yaw)    cos(yaw)];
a_ = (A * (a)')' + xc;
p = patch(a_(:,1), a_(:,2),color);
hold on
h = fill(X,Y,'y');
% plot(X, Y,'color','r', 'linewidth',2)
set(h,'EdgeColor','y','edgealpha',1,'facealpha',0.3)
end
