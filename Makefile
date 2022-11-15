ARM:=arm-none-eabi
CFLAGS:=-O2 -g -mthumb -I Libreria
OBJS:=main.o vector_table.o

all: out.bin out.lis

out.bin: out.elf
	$(ARM)-objcopy -O binary $< $@

out.lis: out.elf
	$(ARM)-objdump -S $< > $@

out.elf: $(OBJS)
	$(ARM)-ld -o $@ -T linker.ld $(OBJS)

%.o: %.s
	$(ARM)-as -o $@ -c $< 
%.o: %.c
	$(ARM)-gcc -o $@ $(CFLAGS) -c $< 


.PHONY: flash
flash: out.bin
	st-flash write out.bin 0x8000000

.PHONY: debug
debug: out.lis flash
	st-util&
	$(ARM)-gdb out.elf -ex "target extended-remote :4242"
	killall st-util

.PHONY: clean
clean:
	rm -rf $(OBJS) out.elf out.bin out.lis

