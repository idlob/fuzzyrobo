%drawbot - plots the robot & its wiskers on the course
%  This draws the robot so that it appears to scamper about on the course.
%  This makes it the simulation fun to watch and addictive to improve.
%  Inputs:
%    posn - [yposn, xposn, theta]
%    rad - radius of the robot's body
%    course - obstacle matrix (0 = obstacle ~0 = clear)
%    varargin - {1} targetn [ytargetn, xtargetn, reserved]
%  Outputs: none

function drawbot(posn, rad, course, varargin)
hold off;
%show the course
imagesc(course), axis image off; colormap gray;
hold on;
%draw a little circle for the robot
angs = 0:pi/10:2*pi;
y = posn(1) + rad*sin(angs);
x = posn(2) + rad*cos(angs);
plot(y,x);
%draw 10cm 'DANGER ZONE'
% y = posn(1) + (rad+10)*sin(angs);
% x = posn(2) + (rad+10)*cos(angs);
% plot(y,x,'r:');
%draw little wiskers representing 10cm rangefinders
drawang([posn(1), posn(2), posn(3) + pi/8],10+rad);
drawang([posn(1), posn(2), posn(3) - pi/8],10+rad);
%draw target zone, if targetn passed
if(~isempty(varargin))
    y = varargin{1}(1) + 2*sin(angs);
    x = varargin{1}(2) + 2*cos(angs);
    plot(y,x,'r');
end

%drawang - plots a ray based on starting posn, theta, and magnitude
%  This draws rays coming from the center of the robot.  I use these to
%  help visulize rangefinders.  This is called from drawbot.
%  Inputs:
%    posn - [yposn, mag]
%    mag - length of the line (pixels)
%  Outputs: none
function drawang(posn, mag)
% do some polar to cartesian math
y = [posn(1), posn(1)+ mag*sin(posn(3))];
x = [posn(2), posn(2)+ mag*cos(posn(3))];
%plot the line
plot(y,x,'r');

%setup target
