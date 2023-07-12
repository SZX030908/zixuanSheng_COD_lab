.data
last_tail_ptr:
        .word 0

.globl intr_handler
	.align  2
	.type intr_handler, @function
	.section .exception

intr_handler:

	#lw $k0,last_tail_ptr($0)
#将之前的ptr数值放到k0寄存器中（从内存中）
	#li $k1,0x60020000
	#lw $k1,8($k1)
#将当前的ptr数值放到k1寄存器中

	#sw $k1,last_tail_ptr($0)
#更新ptr

	#sub $k0,$k1,$k0
	#srl $k0,12
#当前减之前，移位得到当前阶段搬运的子缓冲区数

	#lw $k1,16($0)
	#sub $k1,$k1,$k0
	#sw $k1,16($0)
#dma还未搬运的子缓冲区数减去新搬运的子缓冲区数，然后存回

	#li $k0,1
	#li $k1,0x60020000 
	#lw $k1,0x14($k1)
#将ctrl_stat寄存器中的值放到k1中

	#and $k0,$k0,$k1
	#li $k1,0x60020000
	#sw $k0,0x14($k1)
#将k1寄存器中的值和1与，然后存回去（ctrl_stat[31]位拉低，同时dma使能）

	lw	$k1,16($0)
	addiu 	$k1,$k1,-1
	sw	$k1,16($0)
	li	$k1,1
	li	$k0,0x60020000
	sw	$k1,20($k0)

	eret
	# TODO: Please add your own interrupt handler for DMA engine 
