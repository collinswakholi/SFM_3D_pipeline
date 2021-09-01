obj_arm = serial('COM7');
fopen(obj_arm);

fwrite(obj_arm, '#5#O#4');
fwrite(obj_arm, '#5#O#4');

fwrite(obj_arm, '#5#C#4');
fwrite(obj_arm, '#5#C#4');

fclose(obj_arm);