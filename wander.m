% wander.m - navigation behavior for robot simulation
%  written by: Shawn Lankton
%  for: ECE8843 (sort-of) fun (mostly)
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
%
%  Outputs:
%    none
%

function wander(posn, rad, wdia, course, dt)

vL_target = 5;
vR_target = 5;

% Drive Around
for i = 1:dt:1000
    rangeL = rangefinder([posn(1), posn(2), posn(3)+pi/8], rad, course); %left
    rangeR = rangefinder([posn(1), posn(2), posn(3)-pi/8], rad, course); %right
    rangeC = rangefinder(posn, rad,course); %center
    
    
    if(rangeL <= 10 || rangeR <= 10)
        %if both are blocked
        if(rangeL <= 10 && rangeR <=10)
            %turn towards larger if possible
            if(rangeL >= rangeR+2)
                vL = -1;
                vR = 1;
            else if(rangeR >= rangeL+2)
                vL = 1;
                vR = -1;
                else
                    %otherwise turn randomly
                    vL = sign(.5-rand);
                    vR = -vL;
                end
            end

            while(rangeL <= 10 || rangeR <= 10 || rangeC <= 5)
                %keep turning until both are free, and then a little more
                rangeL = rangefinder([posn(1), posn(2), ...
                                      posn(3)+pi/8], rad, course); %left
                rangeR = rangefinder([posn(1), posn(2), ...
                                      posn(3)-pi/8], rad, course); %right
                rangeC = rangefinder(posn, rad,course); %center
                
                posn = drive(posn, wdia, vL, vR, dt);
                
                drawbot(posn,rad,course);
                
                if(detectcollision(posn,rad,course))
                    disp(['Collision Detected at (' num2str(posn(1))...
                          ',' num2str(posn(2)) ')']);
                end
                drawnow;
            end
            %continue with slightly random arc
            vL = vL_target;
            vR = vR_target;
        else
            %if only one is blocked, turn away from it
            if(rangeL >= rangeR)
                vL = 5-11+rangeR;
                vR = 5;
            else
                vL = 5;
                vR = 5-11+rangeL;
            end
        end
    else
        %if nobody is blocked, contintue on the random arc.
        vL = vL_target;
        vR = vR_target;
    end
    
    % every 15 iterations, determine a new random-ish arc
    if(mod(i,5) == 0)
        vL_target = 5;
        vR_target = 5 + 2*(.5-rand);
    end
    
    %determine new position
    posn = drive(posn, wdia, vL, vR, dt);
    if(detectcollision(posn,rad,course))
        disp(['Collision Detected at (' num2str(posn(1)) ...
              ',' num2str(posn(2)) ')']);
    end

    %draw the robot
    drawbot(posn,rad,course);
    drawnow;

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

% get size of course
[dimy, dimx] = size(course);

%for 100 steps.
for i = rad:(100+rad)
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

%if no obstacles found within 100 pixels, return 100
dist = 100;
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

%get speed differences & sums
vdiff = vR-vL;
vsum = vR+vL;

%calculate new angle (pretty simple)
newposn(3) = posn(3) + vdiff*t/wdia;

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

%get a circle representing the robot's body
angs = 0:pi/10:2*pi;
y = posn(1) + rad*sin(angs);
x = posn(2) + rad*cos(angs);

%for each of these 20 pixels, check if one is on black
for i = 1:length(y)
    if(course(round(x(i)),round(y(i))) == 0)
        %if so, return 1
        bool = 1;
        return;
    end
end

%if not, return 0
bool = 0;

