clear;

%% Build Robot and Environment

% Load robot and adjust home configuration
robot = loadrobot("universalUR5e", "DataFormat","row");
robot.Bodies{4}.Joint.HomePosition = 3*pi/2;
config = homeConfiguration(robot);
config = [config 0 0 0 0 0 0];

% Attach end effector

% Load the gripper
gripper = loadrobot("robotiq2F85", "DataFormat","row");

% Add a joint at the end of the arm and fix the gripper to it
efJoint = rigidBodyJoint("efJoint");
setFixedTransform(efJoint,[roty(90, "deg"),[0 0 0]';0 0 0 1])
replaceJoint(robot,'tool0',efJoint);
addSubtree(robot,'tool0',gripper,'ReplaceBase',false);

% Create the environment
bench = collisionBox(0.5, 0.9, 0.05);
belt1 = collisionBox(1.2, 0.7, 0.05);
barricade = collisionBox(0.5, 0.03, 0.35);
cylinder = collisionCylinder(0.03, 0.1);

TBench = trvec2tform([0.35+0.15 0 0.2]);
TBelt1 = trvec2tform([0.15 -0.6 0.2]);
TCyl = trvec2tform([0.52 0 0.278]);

bench.Pose = TBench;
belt1.Pose = TBelt1;
barricade.Pose = trvec2tform([0.3+0.2, -0.25, 0.4]);
cylinder.Pose = TCyl;

env = {bench, belt1, cylinder, barricade};

% vVisualise robot and environment at the initial configuration
startConfig = config;
show(robot,startConfig,"Visuals","on","Collisions","off", "Frames","off");
hold on
for i = 1:length(env)
    show(env{i});
end

%% Create RRT plannar

% Initialise the plannar and set the RRT parameters
planner = manipulatorRRT(robot, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;
planner.EnableConnectHeuristic = true;
planner.IgnoreSelfCollision = true;

% Solve the robot's inverse kinematics for the picking location
initialguess = startConfig;
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1];
[configSoln,solnInfo] = ik('tool0',[rotx(180,"deg")  [0.5 0 0.43]'; 0 0 0 1],weights,initialguess);
goalConfig = configSoln;


%% Plan the path to goto the cylinder

rng('default');
path = plan(planner,startConfig,goalConfig);
interpStates = interpolate(planner, path);

% Visualise the path
D((size(interpStates,1)+1)/2) = struct('cdata',[],'colormap',[]);
for i = 1:2:size(interpStates,1)
    show(robot, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","on",...
        "Collisions","off", "Frames","off");
    title("UR5e Pick and Place with RRT")
    D((i+1)/2) = getframe(gcf);
    drawnow;
end



current_configuration =  configSoln;

%% Pick the cylinder

% Close the gripper
show(robot, current_configuration+[zeros(1,6) 0.1*ones(1,6)],...
    "PreservePlot", false,...
    "Visuals","on",...
    "Collisions","off", "Frames","off");
title("UR5e Pick and Place with RRT")

drawnow;
current_configuration =  current_configuration+[zeros(1,6) 0.1*ones(1,6)];

% Attach the cylinder to the arm's rigid body tree

% Create the can as a rigid body
cylinder = env{3};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");
% addVisual(canBody, "cylinder" ,[0.03 0.1])

% Get current pose of the robot hand.
startConfig = current_configuration;
endEffectorPose = getTransform(robot,startConfig,"tool0");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder,inv(cylinder.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(robot,canBody,"tool0");

% Remove object from environment.
env(3) = [];


%% Plan the path to the desired destination

% Solve the robot's inverse kinematics for the desired destination
initialguess = current_configuration;
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1];
[configSoln,solnInfo] = ik('tool0',[rotx(180,"deg")  [0.25 -0.5 0.43]'; 0 0 0 1],weights,initialguess);
goalConfig = configSoln;

% Initialise the plannar (With the can) and set the RRT parameters
planner = manipulatorRRT(robot, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;
planner.EnableConnectHeuristic = true;
planner.IgnoreSelfCollision = true;

%Plan the path
path = plan(planner,current_configuration,goalConfig);
interpStates = interpolate(planner, path);

% Visualise the path
hold off
show(robot,config,"Visuals","on","Collisions","off","Frames","off");
hold on
for i = 1:length(env)
    show(env{i});
end
E((size(interpStates,1)+1)/2) = struct('cdata',[],'colormap',[]);
for i = 1:2:size(interpStates,1)
    for ii = 1:length(env)
        show(env{ii});
        hold on
    end
    show(robot, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","on",...
        "Collisions","off", "Frames","off");
    title("UR5e Pick and Place with RRT")
    canPose = getTransform(robot,interpStates(i,:),"myCan",'base_link');
    can = collisionCylinder(0.03, 0.1);
    can.Pose = canPose;
    show(can);
    E((i+1)/2) = getframe(gcf);
    drawnow;
    hold off
end
hold on

current_configuration =  configSoln;

%% Place the cylinder

% Open the gripper
show(robot, current_configuration+[zeros(1,12)],...
    "PreservePlot", false,...
    "Visuals","on",...
    "Collisions","off", "Frames","off");
title("UR5e Pick and Place with RRT")

drawnow;
current_configuration =  current_configuration-[zeros(1,6) 0.1*ones(1,6)];

% Get can pose
canPose = getTransform(robot,current_configuration,"myCan",'base_link');

% Place the can
removeBody(robot,"myCan");

% Defin the can as a collision body
cylinder = collisionCylinder(0.03, 0.1);
cylinder.Pose = canPose;

% Add the can back to the environment
env = {env{1}, env{2}, env{3}, cylinder};

%% Go back to initial position

% Initialise the plannar and set the RRT parameters
planner = manipulatorRRT(robot, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;
planner.EnableConnectHeuristic = true;
planner.IgnoreSelfCollision = true;

% Plan the path
rng('default');
path = plan(planner,current_configuration,config+[0.5 zeros(1,11)]);
interpStates = interpolate(planner, path);

% Visualise the path
F((size(interpStates,1)+1)/2) = struct('cdata',[],'colormap',[]);
for i = 1:2:size(interpStates,1)
    show(robot, interpStates(i,:),...
        "PreservePlot", false,...
        "Visuals","on",...
        "Collisions","off", "Frames","off");
    title("UR5e Pick and Place with RRT")
    F((i+1)/2) = getframe(gcf);
    drawnow;
end


% Save the video
v = VideoWriter('myFile','MPEG-4');
open(v)
writeVideo(v,D)
writeVideo(v,E)
writeVideo(v,F)
close(v)

