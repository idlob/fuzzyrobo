%setup.m - Setup for robot simulation
%  Sets up necessary variables and starts the simulation running.  Some
%  variables are set in the file (robot radius, wheel diameter, timestep,
%  course image).  Position is set dynamically each time setup is called
%  based on two mouse-clicks by the user.  No error checking is included,
%  so don't put points that put the robot outside of the course or in an
%  obstacle.

WANDER_MODE = 'fuzzy';  %(other otpion >>WANDER_MODE = 'default';fuzzy)
COURSE_NAME = 'offic';  %(other option >>COURSE_NAME = 'office';course)
GO = 1;                 %goal-oriented control

posn = [0,0,0,0];         %ydim, xdim, angle
targetn = [];      %ydim, xdim, reserved
rad = 4;                %robot's body radius (pixels)
wdia = 7;               %distance between robot's wheels (pixels)
dt = .5;                %timestep between driving and colecting sensor data

if(strcmp(COURSE_NAME,'course'))
  course = rgb2gray(imread('course.png'));  %image of 'course'
else
  course = rgb2gray(imread('office.png'));  %image of 'office'
end

imagesc(course), axis image off; colormap gray;

title('Click to specify robots starting position');
%collect input point for robot starting posn.
[posn(1) posn(2)] = ginput(1);
drawbot(posn,rad, course);

title('Click to specify robots inital heading');
%collect input point for robot heading
[y x] = ginput(1);
y = y - posn(1);
x = x - posn(2);
[posn(3), r] = cart2pol(x,y);
posn(4) = posn(3);

%show the start position for half a second
drawbot(posn, rad, course);

if(GO)
    title('Click to specify robots target position');
    %collect input point for robot target posn.
    [targetn(1) targetn(2)] = ginput(1);

    %show the start and target position
    drawbot(posn, rad, course, targetn);
end

pause(0.5);

%begin to wander
if(strcmp(WANDER_MODE,'fuzzy'))
  if(GO) 
      wander_fuzzy(posn, rad, wdia, course, dt, targetn);
  else
      wander_fuzzy(posn, rad, wdia, course, dt);
  end;
else
  wander(posn, rad, wdia, course, dt);
end
