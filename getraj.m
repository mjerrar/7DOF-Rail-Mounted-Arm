% dhparams = [0   	pi/2    0.675   0;
%             0.35	pi/2      0     0;
%             1.15	0       0		0;
%             0.752     0       0   0;
%             0       pi/2    0   	0;
%             0       0   0.215  0];

% Define Robot Object
KUKA = robotics.RigidBodyTree('MaxNumBodies',7);
% Define Body and Joint Objects
body1 = robotics.RigidBody('body1');
jnt1 = robotics.Joint('jnt1','prismatic');
body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
body3 = robotics.RigidBody('body3');
jnt3 = robotics.Joint('jnt3','revolute');
body4 = robotics.RigidBody('body4');
jnt4 = robotics.Joint('jnt4','revolute');
body5 = robotics.RigidBody('body5');
jnt5 = robotics.Joint('jnt5','revolute');
body6 = robotics.RigidBody('body6');
jnt6 = robotics.Joint('jnt6','revolute');
body7 = robotics.RigidBody('body7');
jnt7 = robotics.Joint('jnt7','revolute');


% Move Z 500 Rotate Y90
tform=makehgtform('translate',[0 0 0.5],'yrotate',pi/2,'xrotate',-pi/2);
setFixedTransform(jnt1,tform);
% Move X -215.6 Rotate Y-90
tform=makehgtform('translate',[-0.2156 0 0],'yrotate',-pi/2);
setFixedTransform(jnt2,tform);
%Move X -675+50 Y 350 Rotate Z90 Y180 
tform=makehgtform('translate',[0 -0.35 0.4594],'yrotate',pi/2,'zrotate',-pi/2);
setFixedTransform(jnt3,tform);
%Move X 1150
tform=makehgtform('translate',[1.15 0 0]);
setFixedTransform(jnt4,tform);
%Move X 752 Y 41 Rotate Y90
tform=makehgtform('translate',[0.752 0.041 0],'yrotate',pi/2);
setFixedTransform(jnt5,tform);
%Move Z 438 Rotate X90
tform=makehgtform('translate',[0 0 0.448],'xrotate',pi/2);
setFixedTransform(jnt6,tform);
%Move Y -207
tform=makehgtform('translate',[0 0.207 0],'xrotate',-pi/2);
setFixedTransform(jnt7,tform);


body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;

addBody(KUKA,body1,'base')
addBody(KUKA,body2,'body1')
addBody(KUKA,body3,'body2')
addBody(KUKA,body4,'body3')
addBody(KUKA,body5,'body4')
addBody(KUKA,body6,'body5')
addBody(KUKA,body7,'body6')

KUKA.Bodies{1,1}.Joint.PositionLimits=[0 10];

KUKA.DataFormat = 'row';
%%
Jacobian=geometricJacobian(KUKA,KUKA.homeConfiguration,'body7')

%%
gripper = 'body7';

cupHeight = 0.2;
cupRadius = 0.05;
cupPosition = [1, 2, 1];

cup_body = robotics.RigidBody('cupFrame');
setFixedTransform(cup_body.Joint, trvec2tform(cupPosition))
addBody(KUKA, cup_body, 'base');

numWaypoints = 5;
q0 = homeConfiguration(KUKA);
qWaypoints = repmat(q0, numWaypoints, 1);

heightAboveTable = robotics.CartesianBounds('body7');
heightAboveTable.Bounds = [-inf, inf; ...
                           -inf, inf; ...
                           0,inf];
                       
distanceFromCup = robotics.PositionTarget('cupFrame');
distanceFromCup.ReferenceBody = gripper;
distanceFromCup.PositionTolerance = 0.005

alignWithCup = robotics.AimingConstraint('body7');
alignWithCup.TargetPoint = [0 0 0]

limitJointChange = robotics.JointPositionBounds(KUKA)

fixOrientation = robotics.OrientationTarget(gripper);
fixOrientation.OrientationTolerance = deg2rad(1)

intermediateDistance = 0

limitJointChange.Weights = ones(size(limitJointChange.Weights));
limitJointChange.Weights(2)=0.1
limitJointChange.Weights(3)=0.5
fixOrientation.Weights = 0;
alignWithCup.Weights = 1;

distanceFromCup.TargetPosition = [0,0,intermediateDistance]



gik = robotics.GeneralizedInverseKinematics('RigidBodyTree', KUKA, ...
    'ConstraintInputs',{'cartesian','position','aiming','orientation','joint'},...
    'SolverAlgorithm','LevenbergMarquardt')

[qWaypoints(2,:),solutionInfo] = gik(q0, heightAboveTable, ...
                       distanceFromCup, alignWithCup, fixOrientation, ...
                       limitJointChange)


fixOrientation.TargetOrientation = ...
    tform2quat(getTransform(KUKA,qWaypoints(2,:),gripper));

finalDistanceFromCup = 0;
distanceFromCupValues = linspace(intermediateDistance, finalDistanceFromCup, numWaypoints-1);

maxJointChange = pi;

for k = 3:numWaypoints
    % Update the target position.
    distanceFromCup.TargetPosition(3) = distanceFromCupValues(k-1);
    % Restrict the joint positions to lie close to their previous values.
    limitJointChange.Bounds = [qWaypoints(k-1,:)' - maxJointChange,...
                               qWaypoints(k-1,:)' + maxJointChange];
    % Solve for a configuration and add it to the waypoints array.
    [qWaypoints(k,:),solutionInfo] = gik(qWaypoints(k-1,:), ...
                                         heightAboveTable, ...
                                         distanceFromCup, alignWithCup, ...
                                         fixOrientation, limitJointChange)
end

framerate = 15;
r = robotics.Rate(framerate);
tFinal = 30;
tWaypoints = [0,linspace(tFinal/2,tFinal,size(qWaypoints,1)-1)];
numFrames = tFinal*framerate;
qInterp = pchip(tWaypoints,qWaypoints',linspace(0,tFinal,numFrames))';

gripperPosition = zeros(numFrames,3);

for k = 1:numFrames
    gripperPosition(k,:) = tform2trvec(getTransform(KUKA,qInterp(k,:), ...
                                                    gripper));
end

figure;
show(KUKA, qWaypoints(1,:), 'PreservePlot', false);
hold on
exampleHelperPlotCupAndTable(cupHeight, cupRadius, cupPosition);
p = plot3(gripperPosition(1,1), gripperPosition(1,2), gripperPosition(1,3),'r');

hold on
for k = 1:size(qInterp,1)
    axis([-10 10 -10 10 -10 10])
    show(KUKA, qInterp(k,:), 'PreservePlot', false);
    p.XData(k) = gripperPosition(k,1);
    p.YData(k) = gripperPosition(k,2);
    p.ZData(k) = gripperPosition(k,3);
    waitfor(r);
end
hold off




