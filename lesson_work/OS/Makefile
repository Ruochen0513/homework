TOOL_PREFIX = x86_64-linux-gnu-

CFLAGS = -g -c -O0 -m32 -fno-pie -fno-stack-protector -nostdlib -nostdinc

all: boot.s
	$(TOOL_PREFIX)gcc $(CFLAGS) boot.s
	$(TOOL_PREFIX)ld -m elf_i386 -Ttext=0x7c00 boot.o -o os.elf
	$(TOOL_PREFIX)objcopy -O binary os.elf os.bin
	dd if=os.bin of=disk.img conv=notrunc

clean:
	rm -f *.elf *.o