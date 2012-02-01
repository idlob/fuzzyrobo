%wander_fuzzy.m - navigation behavior for robot simulation
%  written by: Shawn Lankton
%  modified by: Ervin Burkus
%  messed up by: Boldizsar Kiss
%  fuzzy controller added by: Ervin Burkus
%
%  This function collects 'rangefinder' data and uses it to determine
%  velocities for the left and right 'wheels'.  It then calls the drive
%  function to predict where those commands will put the robot and
%  continues.  This is the real 'heart' of the simulation.
%
%  Inputs:
%    posn - [yposn, xposn, theta]
%    rad - radius of the robot body
%    wdia - diameter of the robot's wheelbase
%    course - obstacle matrix (0 = obstacle ~0 = clear)
%    dt - timestep to take between driving and collecting sensor data
%    varargin - {1} targetn [ytargetn, xtargetn, reserved]
%
%  Outputs:
%    none
%
function wander_fuzzy(posn, rad, wdia, course, dt, varargin)
robofis = readfis('fuzzyroboErvin.fis');
%vL_target = 5;
%vR_target = 5;
for i = 1:dt:1000 % Drive Around
    rangeL = rangefinder([posn(1), posn(2), posn(3)+pi/8], rad, course); %left
    rangeR = rangefinder([posn(1), posn(2), posn(3)-pi/8], rad, course); %right
    rangeC = rangefinder(posn, rad,course); %center
    
    fin = floor(posn(3) / (2*pi));
    fi = posn(3) - (fin*2*pi);
    
    distY = varargin{1}(2) - posn(2); 
    distX = varargin{1}(1) - posn(1);
    disp([num2str(distY),' ',num2str(distX),' ',num2str(fi*(180/pi)),' ',num2str(fi*(180/pi))]);

    if(rangeL > 10), rangeL = 10; end
    if(rangeR > 10), rangeR = 10; end
    if(rangeC > 10), rangeC = 10; end
	
    out = evalfis([rangeL, rangeC, rangeR], robofis);
    vR = out(1);
    vL = out(2);
	
    posn = drive(posn, wdia, vL, vR, dt); %determine new position
% title('Click to specify robots starting position');
% %collect input point for robot starting posn.
% [posn(1) posn(2)] = ginput(1);
% drawbot(posn,rad, course);
% 
% title('Click to specify robots inital heading');
% %collect input point for robot heading
% [y x] = ginput(1);
% y = y - posn(1);
% x = x - posn(2);
% [posn(3), r] = cart2pol(x,y);

    
    if(detectcollision(posn,rad,course)) %if detects collision displays messeage
        disp(['Collision Detected at (' num2str(posn(1)) ...
        ',' num2str(posn(2)) ')']);
    end

    if(isempty(varargin)) %are we GO?
        drawbot(posn,rad,course); %draw the robot
    else
        drawbot(posn,rad,course,varargin{1}); %draw the robot
    end
    drawnow;
    pause(.5);
end
%rangefinder.m - finds distance to nearest obstacle
%  written by: Shawn Lankton
%  for: ECE8843 (sort-of) fun (mostly)
%
%  The 'sensor' of this robotic simuator.  It starts at the border of the
%  robot and then marches outward, testing each pixel.  If it finds a 0
%  pixel (obstacle) it returns the number of steps taken to get there.
%  Thus it acts like a rangefinder.  It has a max range of 100, and a min
%  range of 0, although if you get a 0 reading it means you crashed.
%
%  Also, this takes in posn in the same way all these other functions do,
%  but it may be beneficial to change the value of posn(3) (theta) so that
%  the rangefinder doesn't point straight forward.  In this way you can
%  simulate multiple ranefinders at different angles.
%
%  Another note... this isn't totally realistic because it doesn't have any
%  field of view.  Its just a line real IR or sonar rangefinders have a x
%  degree field of view and objects anywhere within are seen.  This
%  doesn't.
%
%  Inputs:
%    posn - [yposn, xposn, theta]
%    rad - radius of the robot's body
%    course - obstacle matrix (0 = obstacle ~0 = clear)
%
%  Outputs:
%    dist - distance to obstacle.
%
function dist = rangefinder(posn, rad, course)
[dimy, dimx] = size(course); % get size of course
for i = rad:(100+rad) %for 100 steps.
    %get x&y coords of point to test
    y = posn(1)+ i*sin(posn(3));
    x = posn(2)+ i*cos(posn(3));
    %make sure x & y are inside the course
    x(x < 1) = 1;
    x(x > dimx) = dimx;
    y(y < 1) = 1;
    y(y > dimy) = dimy;
    %see if the course at x & y is an obstacle or not
    if(course(round(x),round(y)) == 0)
        %if yes, return the distance to the obstacle
        dist = i-rad;
        return;
    end
end
dist = 100; %if no obstacles found within 100 pixels, return 100
return;
%drive.m - does the dead rekoning kinematics based on wheel speeds
%  written by: Shawn Lankton
%  for: ECE8843 (sort-of) fun (mostly)
%
%  This predicts how the robot's position will change based on velocities
%  for the left and right wheel along with a 'time driving.'  This is based
%  on the simplest formulation for 2-wheeled kinematics that I could find.
%  http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html
%
%  Inputs:
%    posn - [yposn, xposn, theta]
%    wdia - diameter of the robot's wheelbase
%    vL - left wheel velocity
%    vR - right wheel velocity
%    t - time driving at these velocities
%
%  Outputs:
%    newposn - [yposn, xposn, theta]
%
function [newposn] = drive(posn, wdia, vL, vR, t)
vdiff = vR-vL; %get speed differences & sums
vsum = vR+vL;
newposn(3) = posn(3) + vdiff*t/wdia; %calculate new angle (pretty simple)
newposn(4) = vdiff*t/wdia; %calculate new angle (pretty simple)
if(vdiff == 0)
    %calculate new [y x] if wheels moving together.
    newposn(1) = vL*t*sin(posn(3))+posn(1);
    newposn(2) = vR*t*cos(posn(3))+posn(2);  
else
    %calculate new [y x] if wheels moving at unequal speeds.
    newposn(1) = posn(1) - wdia*vsum/(2*vdiff)*(cos(vdiff*t/wdia+posn(3))-cos(posn(3)));
    newposn(2) = posn(2) + wdia*vsum/(2*vdiff)*(sin(vdiff*t/wdia+posn(3))-sin(posn(3)));
end
%detectcollisions.m - sees if the robot hit something
%  written by: Shawn Lankton
%  for: ECE8843 (sort-of) fun (mostly)
%
%  This just checks the border of the robot to see if any of the pixels are
%  on black (an obstacle).  The function is a little 'iffy' because it has
%  significant rounding error (espicially on my 128x128 course).  Also, if
%  an obstacle somehow gets inside the robot, detectcollisions won't see
%  it.  It only detects collisions on the edge.  it returns 1 for 'yes I
%  hit something' and 0 for 'nah, I'm fine.'
%
%  Inputs:
%    posn - [yposn, xposn, theta]
%    rad - radius of the robot's body
%    course - obstacle matrix (0 = obstacle ~0 = clear)
%
%  Outputs:
%    bool - 1 (yes, I hit something), 0 (nah, I'm fine)
%
function bool = detectcollision(posn, rad, course)
angs = 0:pi/10:2*pi; %get a circle representing the robot's body
y = posn(1) + rad*sin(angs);
x = posn(2) + rad*cos(angs);
for i = 1:length(y) %for each of these 20 pixels, check if one is on black
    if(course(round(x(i)),round(y(i))) == 0)
        bool = 1; %if so, return 1
        return;
    end
end
bool = 0; %if not, return 0
