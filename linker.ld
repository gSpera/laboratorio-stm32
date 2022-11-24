MEMORY {
	flash(rx) : ORIGIN = 0, LENGTH = 256K,
	ram(rwx)   : ORIGIN = 0x20000000, LENGTH = 40K
}

SECTIONS {
	. = 0;

	.text : {
		vector_table.o(.vector_table)
		*(.text)
		*(.text.*)

		*(.rodata)
	} > flash

	DATA_START = .;
	.data : {
		*(.data)
		*(.data*)
	} > ram AT> flash
	DATA_END = .;
	DATA_LOAD_ADDR = LOADADDR(.data);


	. = ORIGIN(ram);
	RAM_START = .;
	.bss : {
		*(.bss)
	} > ram
	. = RAM_START + LENGTH(ram);
	RAM_END = .;
}

