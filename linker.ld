MEMORY {
	flash : ORIGIN = 0, LENGTH = 256K,
	ram   : ORIGIN = 0x20000000, LENGTH = 40K
}

SECTIONS {
	. = 0;

	.text : {
		vector_table.o(.vector_table)
		*(.text)
		*(.text.*)
	} > flash


	. = ORIGIN(ram);
	RAM_START = .;
	. += LENGTH(ram);
	RAM_END = .;
}

