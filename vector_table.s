.extern RAM_END
.extern main
.extern isr_tim6dacunder

.section .vector_table
initial_sp: .word RAM_END
reset_handler: .word main
.org 0x118
tim6_dacunder: .word isr_tim6dacunder