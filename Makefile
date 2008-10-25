CC=gcc

SRC=atsc_psip_section.c diseqc.c dump-vdr.c dump-zap.c lnb.c scan.c section.c 
HED=atsc_psip_section.h diseqc.h dump-vdr.h dump-zap.h lnb.h scan.h section.h list.h
OBJ=atsc_psip_section.o diseqc.o dump-vdr.o dump-zap.o lnb.o scan.o section.o

BIND=/usr/local/bin/
INCLUDE=-I../s2/linux/include

TARGET=scan-s2

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLG) $(OBJ) -o $(TARGET) $(CLIB) 

$(OBJ): $(HED)

install: all
	cp $(TARGET) $(BIND)

uninstall:
	rm $(BIND)$(TARGET)

clean:
	rm -f $(OBJ) $(TARGET) *~

%.o: %.c
	$(CC) $(INCLUDE) -c $< -o $@
