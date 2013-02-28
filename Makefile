INCLUDE=/usr/msp430/
CC=msp430-gcc
LD=msp430-ld
AR=msp430-ar
CFLAGS= -std=gnu99 -W -Os -g -mmcu=msp430f149 
	# -pedantic -Wall

OBJ_FILES=\
	cs8900.o \
	tcpip.o \
	webside.o \
	easyWEB.o

TARGET=easyWEB
all: ${TARGET}.elf

${TARGET}.elf : ${OBJ_FILES}	 Makefile
	${CC} ${CFLAGS} -I${INCLUDE}  ${OBJ_FILES} \
	-o $@

install: ${TARGET}.elf
	sudo mspdebug -j olimex "prog $<"

clean:
	-rm *.elf
	-rm *.o

cs8900.o: cs8900.c cs8900.h
easyWEB.o: easyWEB.c easyweb.h cs8900.c cs8900.h tcpip.c tcpip.h \
  webside.c
easyweb_test.o: easyweb_test.c
fet140_uart01_9600.o: fet140_uart01_9600.c
tcpip.o: tcpip.c tcpip.h cs8900.h
webside.o: webside.c
