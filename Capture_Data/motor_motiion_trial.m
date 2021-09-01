clear all; clc;
    loadlibrary('EziMOTIONPlusRx64', @FAS_EziMOTIONPlusR); 
a=libfunctions('EziMOTIONPlusRx64');

PortN = 5;
Baudrate = 115200;
SlaveN = 0;

ServoS = 1; %for servo on

ServoS = 0; %for servo off
Velocity_INC = 700;
Position_INC1 = 100000;

calllib('EziMOTIONPlusRx64', 'FAS_Connect',int32(PortN), int32(Baudrate));

calllib('EziMOTIONPlusRx64', 'FAS_IsSlaveExist',int32(PortN), int32(SlaveN));
calllib('EziMOTIONPlusRx64', 'FAS_ServoEnable', int32(PortN), int32(SlaveN), int32(ServoS));


running = calllib('EziMOTIONPlusRx64', 'FAS_MoveSingleAxisIncPos', int32(PortN), int32(SlaveN)...
   , int32(Position_INC1), int32(Velocity_INC)); %Run

calllib('EziMOTIONPlusRx64', 'FAS_MoveStop', int32(PortN), int32(SlaveN)); % stop

 calllib('EziMOTIONPlusRx64', 'FAS_MovePause', int32(PortN), int32(SlaveN), int32(Pause));%pausej

 

calllib('EziMOTIONPlusRx64', 'FAS_ServoAlarmReset', int32(PortN), int32(SlaveN));
