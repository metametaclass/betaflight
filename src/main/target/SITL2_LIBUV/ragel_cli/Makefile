all: sitl2_command_line.inc sitl2_command_line sitl2_command_line.dot

%.dot: %.rl
	ragel -V -o $@ $<

%.inc: %.rl
	ragel -G2 -L -o $@ $<

sitl2_command_line: sitl2_command_line.c sitl2_command_line.inc
	gcc -g -o $@ $<


clean: 
	rm sitl2_command_line *.inc *.dot *.exe
