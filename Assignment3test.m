% Function to test and show HW 3 for CISC 472 
%
% created on March 18, 2016 by Eric Braun 10121660

function [] = Assignment3test()
fprintf('\n');
disp('Needle point created with only z-translation: [0,0,40] with 45 degree insertion angle');
[T1,R1,I1] = InverseKinematics([0;0;40;1],45);
fprintf('\n');
disp('Translation about the z-axis:');
disp(T1);
disp('Rotation about the z-axis:');
disp(R1);
disp('Insertion value:')
disp(I1);
P1 = ForwardKinematics(40,0,0,45);
disp('Calculated Point from forward kinematics 40 translation:');
disp(P1);

fprintf('\n');
disp('Needle point created with only rotation about z-axis: [0,1,0] with 45 degree insertion angle');
[T2,R2,I2] = InverseKinematics([0;1;0;1],45);
fprintf('\n');
disp('Translation about the z-axis:');
disp(T2);
disp('Rotation about the z-axis:');
disp(R2);
disp('Insertion value:')
disp(I2);
P2 = ForwardKinematics(0,90,2,45);
disp('Calculated point from forward kinematics 90 rotation:');
disp(P2);

fprintf('\n');
disp('Needle point created with only insertion: [-40,0,40] with 45 degree insertion angle');
[T3,R3,I3] = InverseKinematics([-40;0;40;1],45);
fprintf('\n');
disp('Translation about the z-axis:');
disp(T3);
disp('Rotation about the z-axis:');
disp(R3);
disp('Insertion value:')
disp(I3);
P3 = ForwardKinematics(0,0,sqrt(2*40.^2),45);
disp('Calculated point from forward kinematics 2 * 40^2 insertions:');
disp(P3);

fprintf('\n');
for i = 1:10
    Trans = 40.*rand;
    Rot = 26.56 * (2 * rand - 1);
    Insert = (84.85 - 28.28).* rand + 28.28;
    Point = ForwardKinematics(Trans,Rot,Insert,45);
    [Translation, Rotation, Insertion] = InverseKinematics(Point,45);
    disp('Random Translation: ');
    disp(Trans);
    disp('Calculated inverse kinematics translation along z-axis:');
    disp(Translation);
    fprintf('\n');
    disp('Random Rotation:');
    disp(Rot);
    disp('Calculated inverse kinematics rotation about z-axis:')
    disp(Rotation);
    fprintf('\n');
    disp('Random Insertion:');
    disp(Insert);
    disp('Calculated inverse kinematics insertion length: ');
    disp(Insertion);
end

end