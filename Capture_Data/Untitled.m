
PortN=7;
Baudrate=115200;
SlaveN=0;
motor_speed = 4800;

loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 



calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(PortN), int32(Baudrate));



calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(PortN), int32(SlaveN), int32(1));



% calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisAbsPos', int32(PortN), int32(SlaveN)...
%     , int32(0), int32(50000));

load_motor = calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN),...
                int32(-10000), int32(motor_speed));