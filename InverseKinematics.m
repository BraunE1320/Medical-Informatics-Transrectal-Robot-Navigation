% Function to compute the scalar values of the translatoin about the
% z-axis, rotation about the z-axis, and insertion depth of the needle
% given the needle point in robot frame.
%
% Translation and Insertion are given in mm
% Rotation is given in degrees
%
% Function created on March 16, 2016 by Eric Braun 10121660   


function [Translation,Rotation,Insertion] = InverseKinematics(NeedlePoint,NeedleAngle)
    
    
  % Find the distance between the point and the z-axis
  dist = sqrt(NeedlePoint(1)^2 + NeedlePoint(2)^2);
  
  % Conver needle anngle to radians
  r = degtorad(NeedleAngle);
  
  % By law of right trianges:
  Insertion = dist/sin(r);
  
  % Also by law of right trianles:
  zdist = cos(r) * Insertion;
  
  % The translated amount is z component of the point - the z of the
  % triangle
  Translation = NeedlePoint(3) - zdist;
  
  % Translate the point back
  % We don't really need to do this because we will only take x,y
  % coordinates.
  newPoint = [NeedlePoint(1); NeedlePoint(2); NeedlePoint(3) - Translation;1];
 
  % The point where the needle would lie if it had no rotation
  Point = [-cos(degtorad(90) - r) * Insertion;
                            0;
                            cos(r) * Insertion;
                            1];
  
  % Take the x and y coordinates of two points
  newPoint = [newPoint(1); newPoint(2)];
  Point = [Point(1);Point(2)];
  
  % Using the equation v . w = ||v|| ||w|| cos (angle)
  Rotation = acosd(dot(newPoint,Point)/(norm(newPoint)*norm(Point)));

  % If there is a negative y value for the point, then we have a negative
  % rotation
  if (NeedlePoint(2) < 0)
      Rotation = -(Rotation);
  end
  
end