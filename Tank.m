classdef Tank
    %Tank tankect with BodyTree representation. The tank has a hull, a turret, two countermeasures, and a sensor arrangement. The sensor arrangement can be either a single sensor on top of the hull or four sensors at the corners of the hull.

    properties
        BodyTree
    end

    methods
        function tank = Tank(sensorArragement)
            %Construct an instance of this class. Usage: tnak(1) or tank(4)
            %   Create a new tank tankect. sensorArragement is either 1
            %   (360) or 4 (corners). 
            arguments
                sensorArragement (1,1) {mustBeMember(sensorArragement, [1, 4])} = 1
            end
            % base
            tank.BodyTree = rigidBodyTree;
            tank.BodyTree.DataFormat = 'col';
            tank.BodyTree.BaseName = 'base';

            % hull frame
                baseBody = rigidBody("hull");
                addBody(tank.BodyTree,baseBody,tank.BodyTree.BaseName);
            
            % turret frame
                turretBody = rigidBody("turret");
                turretJoint = rigidBodyJoint("turretJoint","revolute");
                setFixedTransform(turretJoint,trvec2tform([0, 0, 0.5]));
                turretBody.Joint = turretJoint;
                addBody(tank.BodyTree,turretBody,"hull");

            % Countermeasure 1
                cm1Yaw = rigidBody("cm1Yaw");
                cm1YawJoint = rigidBodyJoint("cm1YawJoint","revolute");
                setFixedTransform(cm1YawJoint,trvec2tform([0, 0.5, 0.5]));
                cm1Yaw.Joint = cm1YawJoint;
                addBody(tank.BodyTree,cm1Yaw,"turret");

                cm1Pitch = rigidBody("cm1Pitch");
                cm1PitchJoint = rigidBodyJoint("cm1PitchJoint","revolute");
                setFixedTransform(cm1PitchJoint,trvec2tform([0, 0, 0])); % singularity here 
                cm1Pitch.Joint = cm1PitchJoint;
                addBody(tank.BodyTree,cm1Pitch,"cm1Yaw");

                cm1 = rigidBody("cm1");
                cm1Joint = rigidBodyJoint("cm1Joint","fixed");
                setFixedTransform(cm1Joint,trvec2tform([0, 0.2, 0]));
                cm1.Joint = cm1Joint;
                addBody(tank.BodyTree,cm1,"cm1Pitch");

            % Countermeasure 2
                cm2Yaw = rigidBody("cm2Yaw");
                cm2YawJoint = rigidBodyJoint("cm2YawJoint","revolute");
                setFixedTransform(cm2YawJoint,trvec2tform([0, -0.5, 0.5]));
                cm2Yaw.Joint = cm2YawJoint;
                addBody(tank.BodyTree,cm2Yaw,"turret");

                cm2Pitch = rigidBody("cm2Pitch");
                cm2PitchJoint = rigidBodyJoint("cm2PitchJoint","revolute");
                setFixedTransform(cm2PitchJoint,trvec2tform([0, 0, 0])); % wrist singularity here 
                cm2Pitch.Joint = cm2PitchJoint;
                addBody(tank.BodyTree,cm2Pitch,"cm2Yaw");

                cm2 = rigidBody("cm2");
                cm2Joint = rigidBodyJoint("cm2Joint","fixed");
                setFixedTransform(cm2Joint,trvec2tform([0, -0.2, 0]));
                cm2.Joint = cm2Joint;
                addBody(tank.BodyTree,cm2,"cm2Pitch");

            % Sensors
                if sensorArragement == 1
                    sensor = rigidBody("sensor");
                    sensorJoint = rigidBodyJoint("sensorJoint","fixed");
                    setFixedTransform(sensorJoint,trvec2tform([0, 0, 0.5]));
                    sensor.Joint = sensorJoint;
                    addBody(tank.BodyTree,sensor,"hull");
                elseif sensorArragement == 4
                    for i = 1:4
                        sensor = rigidBody("sensor" + i);
                        sensorJoint = rigidBodyJoint("sensor" + i + "Joint","fixed");
                        angle = (i-1)*pi/2 + pi/4;
                        setFixedTransform(sensorJoint,trvec2tform([0.5*cos(angle), 0.5*sin(angle), 0.5]));
                        sensor.Joint = sensorJoint;
                        addBody(tank.BodyTree,sensor,"hull");
                    end
                end

        end
        function transform = getFrameTransform(tank, frame, configuration)
            arguments
                tank (1,1) Tank
                frame (1,1) string
                configuration = homeConfiguration(tank.BodyTree)
            end
            transform = getTransform(tank.BodyTree, configuration, 'base', frame);
        end
    end
end