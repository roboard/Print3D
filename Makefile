IFLAGS    = -I./ -I./Include -I./86Duino -I./utility
OBJFILES  = main.o io.o irq.o uart.o vortex86.o com.o common.o queue.o usb.o usb_desc.o can.o mcex.o temperature.o command.o protocol.o communication.o qr_solve.o vector_3.o g_code.o planner.o stepper.o global_setting.o wanalog.o Block.o EEPROM.o SPIFlash.o mem_pool.o btree.o ini.o
EXEFILES  = Print3D_.exe
LIBFILES  = -lstdcxx
OPTIONS   = -O3 -Wno-write-strings
CXX       = gxx

.PHONY : everything all clean

everything : $(OBJFILES) $(EXEFILES)

all : clean everything

clean :
	-rm -f $(EXEFILES) $(OBJFILES)

io.o : 86Duino/io.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
irq.o : 86Duino/irq.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
uart.o : 86Duino/uart.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
vortex86.o : 86Duino/vortex86.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
com.o : 86Duino/com.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
common.o : 86Duino/common.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
queue.o : 86Duino/queue.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
usb.o : 86Duino/usb.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
usb_desc.o : 86Duino/usb_desc.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
can.o : 86Duino/can.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
mcex.o : 86Duino/mcex.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
wanalog.o : 86Duino/wanalog.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
Block.o : 86Duino/Block.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
EEPROM.o : 86Duino/EEPROM.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
SPIFlash.o : 86Duino/SPIFlash.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
qr_solve.o : utility/qr_solve.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
vector_3.o : utility/vector_3.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
mem_pool.o : utility/mem_pool.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
btree.o : utility/btree.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
ini.o : utility/ini.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
	
temperature.o : temperature.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
command.o : command.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
protocol.o : protocol.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
communication.o : communication.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
g_code.o : g_code.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
planner.o : planner.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
stepper.o : stepper.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
global_setting.o : global_setting.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
main.o : main.cpp
	$(CXX) -c $< $(IFLAGS) $(OPTIONS)
	
Print3D_.exe : $(OBJFILES)
	$(CXX) -o $@ $(OBJFILES) $(LIBFILES) $(OPTIONS)

	