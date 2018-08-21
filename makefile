KEIL_PATH = C:\Keil_v5\ARM
 
ARMCC = $(KEIL_PATH)\BIN\armcc
ARMASM = $(KEIL_PATH)\BIN\armasm
ARMAR = $(KEIL_PATH)\BIN\armar
ARMLINK = $(KEIL_PATH)\BIN\armlink
FROMELF = $(KEIL_PATH)\BIN\fromelf
 
#################################################
# 编译选项
#################################################
CFLAGS := -c --cpu Cortex-M3 -D__MICROLIB -g -O0 --apcs=interwork
CMACRO :=
ASMFLAGS := --cpu Cortex-M3 -g --apcs=interwork --pd "__MICROLIB SETA 1"
LINKFLAGS := --cpu Cortex-M3 --library_type=microlib --strict
MAP := --autoat --summary_stderr --info summarysizes --map --xref --callgraph --symbols 
INFO := --info sizes --info totals --info unused --info veneers
 
#--cpu Cortex-M3 *.o --library_type=microlib --strict --scatter "TEST.sct" 
#--autoat --summary_stderr --info summarysizes --map --xref --callgraph --symbols 
#--info sizes --info totals --info unused --info veneers 
#--list ".\TEST.map" 
#-o "TEST.axf" 
 
TARGET = .\output\stm32
OBJMAP := .\output\*.map
OBJHTM := .\output\*.htm
OBJAXF := .\output\*.axf
 
OBJS = .\usr\adc7689.o \
		.\usr\boardiodef.o \
        .\system\usart\usart.o \
	    .\hardware\key\key.o \
	    .\hardware\led\led.o \
   	    .\user\test.o \
	    .\user\STM32F10x.o
	   
 
INC += -I.\usr
INC += -I.\system\sys
INC += -I.\system\usart
INC += -I.\hardware\key
INC += -I.\hardware\led
INC += -I$(KEIL_PATH)\INC\St\STM32F10x
INC += -I$(KEIL_PATH)\RV31\INC
 
%.o:%.c
	$(ARMCC) $(CFLAGS) $(INC) $(CMACRO) $< -o $@
	
%.o:%.s
	$(ARMASM) $(ASMFLAGS) $(INC) $(CMACRO) $< -o $@	
 
arm7:$(OBJS)
	$(ARMLINK) $(LINKFLAGS) --libpath "$(KEIL_PATH)\RV31\LIB" --scatter LoaderPrj.sct $(MAP) $(INFO) --list $(TARGET).map $^ -o $(TARGET).axf
	$(FROMELF) --bin -o $(TARGET).bin $(TARGET).axf
	$(FROMELF) --i32 -o $(TARGET).hex $(TARGET).axf
	del $(OBJHTM) $(OBJAXF) $(OBJS)
 
#   若只是生成LIB库，只需要以下一条命令就可以了 
#	$(ARMAR) $(APPNAME).lib -r $(OBJS)
		
.PHONY : clean
 
clean:
	del $(OBJS) *.map *.htm
