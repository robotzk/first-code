clear
clc
close all
N = 10;
axis([-100 500 -100 500]);
figure(1)
x = [0 0
    10 0
    10  1
    6  3
    7 13
    17 8
    17 11
    7 16
    6 21
    4 21
    3 16
    -7 11
    -7 8
    3 13
    4 3
    0 1];
xc = [0 0];
x = x - [5 13];
p = patch(x(:,1),x(:,2),'g');
axis equal
hold on
disp('单击鼠标左键点设定目标点');
disp('单击鼠标右键点结束');
but=1;
o = pi/2;
while but==1
[xi,yi,but]=ginput(1);
if but~=1
    break
end
plot(xi,yi,'ro')
plot([xi xc(1)],[yi xc(2)],'--')
delete(p)
o_ = atan2(yi - xc(2),xi - xc(1));
if o_ < 0
    o_ = 2*pi + o_;
end
oo = o_ - o;
if oo < -pi
    oo  = 2 * pi + oo;
end
if oo > pi
    oo  = oo - 2 * pi;
end
o = o_;
for i = 1:N
    delete(p)
    x_ = (rot_2d(oo*i/N) * (x - xc)')' + xc;
    p = patch(x_(:,1),x_(:,2),'g');
    pause(0.1)
%     axis equal
end
for i = 1:N
    delete(p)
    t = i / N;
    x_t = x_ - xc + t * [xi yi] + (1 - t) * xc;
    p = patch(x_t(:,1),x_t(:,2),'g');
    pause(0.1)
%     axis equal
end
x = x_t;
xc = [xi yi];
disp('单击鼠标左键点取下一个目标点');
end
function A = rot_2d(o)
A = [cos(o)  -sin(o)
    sin(o)    cos(o)];
end