% tank4 = Tank(4);
% show(tank4.BodyTree)
% tform = getFrameTransform(tank4, 'turret')

tank = Tank(1)
threat_gps = [10, 2, 3]
tank.thowaway_example_1(threat_gps);

%Question should we instead do something like target is xyz from CM mount frame, then give the 2 Q values to gimble slew the CM to point at target? - Wesley
