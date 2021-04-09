all: sitl2_command_line.inc sitl2_command_line sitl2_command_line.dot

%.dot: %.rl
	ragel -V -o $@ $<

%.inc: %.rl
	ragel -T1 -L -o $@ $<

sitl2_command_line: sitl2_command_line.c sitl2_command_line.inc
	gcc -g -Wall -Wpedantic -Wextra -o $@ $<


clean: 
	rm -f sitl2_command_line *.inc *.dot *.exe

run: sitl2_command_line
	./sitl2_command_line
