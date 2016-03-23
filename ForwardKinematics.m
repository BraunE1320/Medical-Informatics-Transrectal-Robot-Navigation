% Function to compute the needlepoint tip in 3D for transrectal robot
% navigation given three scalars for translation about z-axis, rotation
% about z-axis and depth the needle goes in.
%
% Translation and Insertion given in mm
% Rotation given in degrees
%
% Function created on March 16th, 2016 by Eric Braun 10121660


function [NeedlePoint] = ForwardKinematics(Translation, Rotation, Insertion, NeedleAngle)
    
    % Convert degrees to radians
    r = degtorad(Rotation);
    insertionAngle = degtorad(NeedleAngle);
    
    % Find where point would be after rotation and insertion 
    Point = [-cos(degtorad(90) - insertionAngle) * Insertion;
                            0;
                            cos(insertionAngle) * Insertion;
                            1];

    % Create rotation matrix
    RotationMatrix = [cos(r), sin(r), 0, 0;
                     -sin(r), cos(r), 0, 0;
                        0   ,   0   , 1, 0;
                        0   ,   0   , 0, 1];
    
    % Create Translation matrix
    Translation = [1, 0, 0, 0;
                   0, 1, 0, 0;
                   0, 0, 1, Translation;
                   0, 0, 0, 1];
  
    % Transform the point by the translation and rotation matrix.
     NeedlePoint = Translation * RotationMatrix * Point;
       
end