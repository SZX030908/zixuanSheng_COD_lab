`timescale 10ns / 1ns

module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:{32, {32-bit),  
*   retired PC                  (31: 0, {32-bit)
* }
*
*/
 	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;
	//有关寄存器的信号

	wire [31:0]		PC_next;
	wire [3:0]		PC_src;
	wire			PC_silent;
	wire [31:0] 	PC_not_beq;
	wire [31:0]		PC_beq_JAL;
	wire [31:0] 	PC_JALR;
	wire			PC_4_valid;


	wire[63:0] 		mul_result;	
	

	reg [31:0] cycle_cnt;//总周期数
	//性能计数器

	wire 	pipeline;	
	wire	pipeline_IF_not_nop;
	wire 	pipeline_MEM_not_nop;

	wire	nop_IF_IDLE;

	wire[31:0]	instruction_reg;
	wire[31:0]	MEMdata_reg;

	wire[3:0]	MEM_current_state;

	/*reg[15:0]	bomb;
	wire 	 	stop;*/


	always @ (posedge clk)
	begin
			if(rst) begin
					cycle_cnt <= 32'b0;
			end
			else begin
					cycle_cnt <= cycle_cnt + 32'b1;
			end
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	IF_state_change IF_state_change0(
		.clk(clk),
		.rst(rst),
		.pipeline(pipeline),
		.instruction(Instruction),

		.Inst_Req_Ready(Inst_Req_Ready),
		.Inst_Valid(Inst_Valid),

		.MEM_en_EX_MEM(MEM_en_EX_MEM),
		.nop_EX_MEM(nop_EX_MEM),
		.MemRead_EX_MEM(MemRead_EX_MEM),
		.MemWrite_EX_MEM(MemWrite_EX_MEM),
		.MEM_current_state(MEM_current_state),

		.instruction_reg(instruction_reg),
		.pipeline_IF_not_nop(pipeline_IF_not_nop),
		.Inst_Req_Valid(Inst_Req_Valid),
		.Inst_Ready(Inst_Ready)
	);

	MEM_state_change MEM_state_change0(
		.clk(clk),
		.rst(rst),
		.pipeline(pipeline),
		.Read_data(Read_data),

		.Mem_Req_Ready(Mem_Req_Ready),
		.Read_data_Valid(Read_data_Valid),

		.MemWrite_EX_MEM(MemWrite_EX_MEM),
		.nop_EX_MEM(nop_EX_MEM),
		.MEM_en_EX_MEM(MEM_en_EX_MEM),
		.MemRead_EX_MEM(MemRead_EX_MEM),
		.MEM_current_state(MEM_current_state),

		.MEMdata_reg(MEMdata_reg),
		.pipeline_MEM_not_nop(pipeline_MEM_not_nop),
		.MemWrite(MemWrite),
		.MemRead(MemRead),
		.Read_data_Ready(Read_data_Ready)
	);

	pipeline_set pipeline_set0(
		.nop_IF_IDLE(nop_IF_IDLE),
		.nop_IF_ID(nop_IF_ID),
		.nop_ID_EX(nop_ID_EX),
		.nop_EX_MEM(nop_EX_MEM),
		.nop_MEM_WB(nop_MEM_WB),
		.MEM_en_EX_MEM(MEM_en_EX_MEM),

		.pipeline_IF_not_nop(pipeline_IF_not_nop),
		.pipeline_MEM_not_nop(pipeline_MEM_not_nop),

		.pipeline(pipeline)
	);

	

	IF_IDLE IF_IDLE0(
		.clk(clk),
		.rst(rst),
		.pipeline(pipeline),

		.PC_next(PC_next),

		.PC(PC),
		.nop_IF_IDLE(nop_IF_IDLE)
	);

/////////////////////////////////////////IF阶段信号	
	wire[31:0]		PC;
	wire			nop_IF;
/////////////////////////////////////////IF阶段信号	

/////////////////////////////////////////IF_ID阶段信号	
	wire[31:0]		PC_IF_ID;
	wire			nop_IF_ID;
	wire[31:0]		instruction_IF_ID;
/////////////////////////////////////////IF_ID阶段信号	

	IF_ID_registers IF_ID_registers0(
		.clk(clk),
		.rst(rst),
		.pipeline(pipeline),

		.nop_IF(nop_IF),//由冒险阻塞单元给出的是否置为nop的信号
		.nop_IF_IDLE(nop_IF_IDLE),
		.instruction_IF(instruction_reg),
		.PC(PC),

		.PC_IF_ID(PC_IF_ID), 
		.nop_IF_ID(nop_IF_ID),
		.instruction_IF_ID(instruction_IF_ID)
	);

//////////////////////////////////////////////////下面进入译码阶段

/////////////////////////////////////////ID阶段信号	
	wire[31:0]		Read_data1;
	wire[31:0]		Read_data2;

	wire			EX_en_ID;
	wire			MEM_en_ID;
	wire			WB_en_ID;

	wire			Branch_ID;
	wire[31:0]		read_dataA_ID;
	wire[31:0]		read_dataB_ID;
	wire      		MemRead_ID;
	wire     		MemtoReg_ID;
	wire       		MemWrite_ID;
	wire[3:0] 		alu_shifter_mul_imm_ID;
	wire	   	 	RegWrite_ID;

	wire[7:0]		pre_ALUOP_ID;
	wire[31:0]		imm_ID;
	wire[31:0]		store_data_ID;
	wire[31:0]		PC_src_data_ID;
/////////////////////////////////////////ID阶段信号	

	reg_file reg0 (
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(instruction_IF_ID[19:15]),
		.raddr2(instruction_IF_ID[24:20]),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(Read_data1),
		.rdata2(Read_data2)
	);


	control control0(
		.instruction(instruction_IF_ID),
		.PC(PC_IF_ID),
		.Read_data1(Read_data1),
		.Read_data2(Read_data2),

		.EX_en_ID(EX_en_ID),
		.MEM_en_ID(MEM_en_ID),
		.WB_en_ID(WB_en_ID),

		.Branch_ID(Branch_ID),
		.read_dataA_ID(read_dataA_ID),
		.read_dataB_ID(read_dataB_ID),
		.MemRead_ID(MemRead_ID),//无效：不读内存；有效：读内存
		.MemtoReg_ID(MemtoReg_ID),//无效：ALU算的数->reg；有效：访存->reg
		.MemWrite_ID(MemWrite_ID),//无效：不写内存；有效：写内存
		.alu_shifter_mul_imm_ID(alu_shifter_mul_imm_ID),//用独热码表示接下来参与运算的数据会进入哪个运算器进行运算
		//0位对应不参加运算，直接放入
		//1位对应参加乘法运算
		//2位对应参加移位运算
		//3位对应参加alu运算
		.RegWrite_ID(RegWrite_ID),//无效：不写寄存器；有效：写寄存器

		.pre_ALUop(pre_ALUOP_ID),
		.imm(imm_ID),
		.store_data_ID(store_data_ID),
		.PC_src_data(PC_src_data_ID)
	);

/////////////////////////////////////////ID_EX阶段信号	
	wire[31:0]		PC_ID_EX;
	wire 			nop_ID_EX;

	wire			EX_en_ID_EX;
	wire			MEM_en_ID_EX;
	wire			WB_en_ID_EX;

	wire			Branch_ID_EX;
	wire[31:0]		read_dataA_ID_EX;
	wire[31:0]		read_dataB_ID_EX;
	wire      		MemRead_ID_EX;
	wire     		MemtoReg_ID_EX;
	wire       		MemWrite_ID_EX;
	wire[3:0] 		alu_shifter_mul_imm_ID_EX;
	wire	    	RegWrite_ID_EX;

	wire[31:0]		instruction_ID_EX;
	wire[7:0]		pre_ALUOP_ID_EX;
	wire[31:0]		imm_ID_EX;
	wire[31:0]		store_data_ID_EX;
	wire[31:0]		PC_src_data_ID_EX;
/////////////////////////////////////////ID_EX阶段信号	

 	ID_EX_registers ID_EX_registers0(
		.pipeline(pipeline),
		.clk(clk),
		.rst(rst),

		.nop_ID(nop_ID),//由冒险阻塞单元给出的是否置为nop的信号
		.nop_IF_ID(nop_IF_ID),//上一级的nop寄存器
		.PC_IF_ID(PC_IF_ID),
		.instruction_IF_ID(instruction_IF_ID),
		.pre_ALUOP_ID(pre_ALUOP_ID),
		.imm_ID(imm_ID),
		.store_data_ID(store_data_ID),
		.PC_src_data_ID(PC_src_data_ID),
 
		.EX_en_ID(EX_en_ID),
		.MEM_en_ID(MEM_en_ID),
		.WB_en_ID(WB_en_ID),

		.Branch_ID(Branch_ID),
		.read_dataA_ID(read_dataA_ID),
		.read_dataB_ID(read_dataB_ID),
		.MemRead_ID(MemRead_ID),
		.MemtoReg_ID(MemtoReg_ID),
		.MemWrite_ID(MemWrite_ID),
		.alu_shifter_mul_imm_ID(alu_shifter_mul_imm_ID),
		.RegWrite_ID(RegWrite_ID),


		.PC_ID_EX(PC_ID_EX),
		.nop_ID_EX(nop_ID_EX),

		.EX_en_ID_EX(EX_en_ID_EX),
		.MEM_en_ID_EX(MEM_en_ID_EX),
		.WB_en_ID_EX(WB_en_ID_EX),

		.Branch_ID_EX(Branch_ID_EX),
		.read_dataA_ID_EX(read_dataA_ID_EX),
		.read_dataB_ID_EX(read_dataB_ID_EX),
		.MemRead_ID_EX(MemRead_ID_EX),
		.MemtoReg_ID_EX(MemtoReg_ID_EX),
		.MemWrite_ID_EX(MemWrite_ID_EX),
		.alu_shifter_mul_imm_ID_EX(alu_shifter_mul_imm_ID_EX),
		.RegWrite_ID_EX(RegWrite_ID_EX),

		.instruction_ID_EX(instruction_ID_EX),
		.pre_ALUOP_ID_EX(pre_ALUOP_ID_EX),
		.imm_ID_EX(imm_ID_EX),
		.store_data_ID_EX(store_data_ID_EX),
		.PC_src_data_ID_EX(PC_src_data_ID_EX)
	);

//////////////////////////////////////////////////下面进入执行阶段

/////////////////////////////////////////EX阶段信号	
	wire [2:0] 		ALUOP;//输出ALUOP
	wire [1:0] 		shiftOP;//输出shiftOP
	//第二级译码

	wire [31:0] 	Result_shift;//用在shifter中进行计算
	wire [31:0] 	Result_ALU;//用在ALU中进行计算
	wire 			Zero;
	wire 			CarryOut;
	wire 			Overflow;

	wire 			branch_judge;
	wire 			exit;//已经确定一定会跳转
	wire [31:0] 	result_EX;//除了B型指令，往后走会在写回阶段放在寄存器中的数据

	wire [31:0]		operand_dataA;
	wire [31:0]		operand_dataB;
	wire [31:0]		store_data_EX;
	wire [31:0]		PC_src_data_EX;

	wire[3:0]		Forward_select_dataA;
	wire[3:0]		Forward_select_dataB;
	wire[3:0]		Forward_select_store_data;
	wire[3:0]		Forward_select_PC_src_data;//决定是否旁路的信号
/////////////////////////////////////////EX阶段信号

///////////////////////////////////通过已有的数据进行运算

	ALU_Shifter_control alu_shifter_control(
		.ALUop_temp(instruction_ID_EX[14:12]),
		.pre_ALUop(pre_ALUOP_ID_EX),
		.judge_1(instruction_ID_EX[30]),

		.ALUOP(ALUOP),
		.shiftOP(shiftOP)
	);

	Forwarding_unit Forwarding_unit0(
		.instruction_EX_MEM(instruction_EX_MEM),
		.instruction_MEM_WB(instruction_MEM_WB),
		.RF_waddr_retire(RF_waddr_retire),

		.RegWrite_EX_MEM(RegWrite_EX_MEM),
		.RegWrite_MEM_WB(RegWrite_MEM_WB),
		.RF_en_rt_retire(RF_en_rt_retire),

		.nop_EX_MEM(nop_EX_MEM),
		.nop_MEM_WB(nop_MEM_WB),
		.nop_retire(nop_retire),

		.instruction_ID_EX(instruction_ID_EX),
		.pre_ALUOP_ID_EX(pre_ALUOP_ID_EX),

		.Forward_select_dataA(Forward_select_dataA),
		.Forward_select_dataB(Forward_select_dataB),
		.Forward_select_store_data(Forward_select_store_data),
		.Forward_select_PC_src_data(Forward_select_PC_src_data)
	);

	assign operand_dataA =  ({32{Forward_select_dataA[0]}} & RF_wdata_retire) |
							({32{Forward_select_dataA[1]}} & ((MemtoReg_MEM_WB) ? MemtoReg_Data_MEM_WB : result_MEM_WB)) |
							({32{Forward_select_dataA[2]}} & result_EX_MEM) |
							({32{Forward_select_dataA[3]}} & read_dataA_ID_EX);

	assign operand_dataB =  ({32{Forward_select_dataB[0]}} & RF_wdata_retire) |
							({32{Forward_select_dataB[1]}} & ((MemtoReg_MEM_WB) ? MemtoReg_Data_MEM_WB : result_MEM_WB)) |
							({32{Forward_select_dataB[2]}} & result_EX_MEM) |
							({32{Forward_select_dataB[3]}} & read_dataB_ID_EX);

	assign store_data_EX =  ({32{Forward_select_store_data[0]}} & RF_wdata_retire) |
							({32{Forward_select_store_data[1]}} & ((MemtoReg_MEM_WB) ? MemtoReg_Data_MEM_WB : result_MEM_WB)) |
							({32{Forward_select_store_data[2]}} & result_EX_MEM) |
							({32{Forward_select_store_data[3]}} & store_data_ID_EX);
	
	assign PC_src_data_EX = ({32{Forward_select_PC_src_data[0]}} & RF_wdata_retire) |
							({32{Forward_select_PC_src_data[1]}} & ((MemtoReg_MEM_WB) ? MemtoReg_Data_MEM_WB : result_MEM_WB)) |
							({32{Forward_select_PC_src_data[2]}} & result_EX_MEM) |
							({32{Forward_select_PC_src_data[3]}} & PC_src_data_ID_EX);

	//0号对应最后一级（最不优先）
	//1号对应MEM_WB级
	//2号对应EX_MEM级
	//3号对应ID_EX级	

	jump_nop_PC jump_nop_PC0(
		.exit(exit),
		.pre_ALUOP_ID(pre_ALUOP_ID),

		.nop_IF_ID(nop_IF_ID),

		.nop_IF(nop_IF),
		.nop_ID(nop_ID),
		.PC_silent(PC_silent)
	);

	shifter shifter0(
		.A(operand_dataA),
		.B(operand_dataB[4:0]),
		.Shiftop(shiftOP),

		.Result(Result_shift)
	);

	ALU alu(
		.A(operand_dataA),
		.B(operand_dataB),
		.ALUop(ALUOP),

		.Overflow(Overflow),
		.CarryOut(CarryOut),
		.Zero(Zero),
		.Result(Result_ALU)
	);

	assign mul_result = operand_dataA * operand_dataB;

	branch_judge branch_judge0(
		.Result(Result_ALU),
		.Judge_op(instruction_ID_EX[14:12]),

		.Zero(Zero),
		.CarryOut(CarryOut),
		.Overflow(Overflow),

		.branch_judge(branch_judge)
	);
//对于可能PC的计算，以及计算是否需要跳转
	assign PC_beq_JAL = PC_ID_EX + imm_ID_EX;//相对寻址
	assign PC_JALR 	  = PC_src_data_EX + imm_ID_EX;//寄存器间接寻址
	assign PC_not_beq = PC + 4;//无跳转

	assign PC_4_valid = nop_ID_EX & nop_EX_MEM & nop_MEM_WB;
	//主要是考虑到最开始的时候后面的流水段都还是不定态，无法进行判断，所以后面均为nop时强行为PC+4

	assign PC_src[0] = !nop_ID_EX & Branch_ID_EX & pre_ALUOP_ID_EX[4] & !instruction_ID_EX[3];
	assign PC_src[1] = !nop_ID_EX & Branch_ID_EX & ((pre_ALUOP_ID_EX[0] & branch_judge) | (pre_ALUOP_ID_EX[4] & instruction_ID_EX[3]));
	assign PC_src[2] = (!PC_src[0] & !PC_src[1] & !PC_src[3]) | PC_4_valid;
	assign PC_src[3] = PC_silent & !PC_src[0] & !PC_src[1];

	assign PC_next 	= 	({32{PC_src[0]}} & PC_JALR) |
						({32{PC_src[1]}} & PC_beq_JAL) |
						({32{PC_src[2]}} & PC_not_beq) |
						({32{PC_src[3]}} & PC) ;

	assign exit = !nop_ID_EX & ((Branch_ID_EX & pre_ALUOP_ID_EX[4]) | (Branch_ID_EX & pre_ALUOP_ID_EX[0] & branch_judge));
	//这里全部都是在对是否跳转，以及跳转地址进行计算

	assign result_EX =  ({32{alu_shifter_mul_imm_ID_EX[0]}} & imm_ID_EX) | 
						({32{alu_shifter_mul_imm_ID_EX[1]}} & mul_result[31:0]) |
						({32{alu_shifter_mul_imm_ID_EX[2]}} & Result_shift) |
						({32{alu_shifter_mul_imm_ID_EX[3]}} & Result_ALU) ;


/////////////////////////////////////////EX_MEM阶段信号	
	wire[31:0]		PC_EX_MEM;
	wire 			nop_EX_MEM;

	wire			MEM_en_EX_MEM;
	wire			WB_en_EX_MEM;

	wire[31:0]		result_EX_MEM;
	wire      		MemRead_EX_MEM;
	wire     		MemtoReg_EX_MEM;
	wire       		MemWrite_EX_MEM;
	wire	    	RegWrite_EX_MEM;

	wire[31:0]		store_data_EX_MEM;
	wire[31:0]		instruction_EX_MEM;
/////////////////////////////////////////EX_MEM阶段信号	

	EX_MEM_registers EX_MEM_registers0(
		.pipeline(pipeline),
		.clk(clk),
		.rst(rst),

		.nop_ID_EX(nop_ID_EX),//上一级的nop寄存器
		.PC_ID_EX(PC_ID_EX),
		.store_data_ID_EX(store_data_EX),
		.instruction_ID_EX(instruction_ID_EX),
	
 
		.MEM_en_ID_EX(MEM_en_ID_EX),
		.WB_en_ID_EX(WB_en_ID_EX),

		.result_EX(result_EX),
		.MemRead_ID_EX(MemRead_ID_EX),//无效：不读内存；有效：读内存
		.MemtoReg_ID_EX(MemtoReg_ID_EX),//无效：ALU算的数->reg；有效：访存->reg
		.MemWrite_ID_EX(MemWrite_ID_EX),//无效：不写内存；有效：写内存
		.RegWrite_ID_EX(RegWrite_ID_EX),//无效：不写寄存器；有效：写寄存器


		.nop_EX_MEM(nop_EX_MEM),

		.MEM_en_EX_MEM(MEM_en_EX_MEM),
		.WB_en_EX_MEM(WB_en_EX_MEM),

		.result_EX_MEM(result_EX_MEM),
		.MemRead_EX_MEM(MemRead_EX_MEM),
		.MemtoReg_EX_MEM(MemtoReg_EX_MEM),
		.MemWrite_EX_MEM(MemWrite_EX_MEM),
		.RegWrite_EX_MEM(RegWrite_EX_MEM),

		.PC_EX_MEM(PC_EX_MEM),
		.store_data_EX_MEM(store_data_EX_MEM),
		.instruction_EX_MEM(instruction_EX_MEM)
	);

//////////////////////////////////////////////////下面进入MEM阶段

/////////////////////////////////////////MEM阶段信号	

	wire [3:0] 		Mask;//用在ALU中进行计算得到从内存中取的地址
	//以上两者均需要ALU的结果
	wire [31:0] 	MemtoReg_Data_MEM;//通过address中取出的数，制作出应该放进rt寄存器种的数

/////////////////////////////////////////MEM阶段信号

	assign Address = {result_EX_MEM[31:2],2'b00};
	Create_Mask create_mask(
		.Result(result_EX_MEM),
		.mask_op(instruction_EX_MEM[14:12]),

		.mask(Mask)
	);

	MemtoReg_Data_Read memtoreg_data_read(
		.mask(Mask),
		.Memdata(MEMdata_reg),
		.mask_op(instruction_EX_MEM[14:12]),

		.memtoReg_data(MemtoReg_Data_MEM)
	);

	Create_Strb create_strb(
		.Result(result_EX_MEM),
		.strb_op(instruction_EX_MEM[14:12]),

		.strb(Write_strb)
	);

	RegtoMem_Data_Write regtomem_data_write(
		.strb(Write_strb),
		.rt_reg_data(store_data_EX_MEM),
	 	.strb_op(instruction_EX_MEM[14:12]),

 		.Regtomem_data(Write_data)
	);


	/*always @(posedge clk) begin
		mem_data_reg <= Read_data;
    end

	always @(posedge clk) begin
		ALU_out_next <= ALU_out;
    end*/
	//在内存访问阶段，通过已有的ALU_out来给出是否该写入，写入的数据，或者把读出的数据放入mem_data_reg寄存器中
	//相对应的，ALU_out放入ALU_out_next中，并且把抹零后剩下的信息放到新的寄存器里

/////////////////////////////////////////MEM_WB阶段信号	

	wire[31:0]		PC_MEM_WB;
	wire 			nop_MEM_WB;

	wire			WB_en_MEM_WB;

	wire[31:0]		result_MEM_WB;
	wire[31:0]		MemtoReg_Data_MEM_WB;
	wire     		MemtoReg_MEM_WB;
	wire	    	RegWrite_MEM_WB;

	wire[31:0]		instruction_MEM_WB;

/////////////////////////////////////////MEM_WB阶段信号	


	MEM_WB_registers MEM_WB_registers0(
		.clk(clk),
		.rst(rst),
		.pipeline(pipeline),

		.nop_EX_MEM(nop_EX_MEM),//上一级的nop寄存器
		.instruction_EX_MEM(instruction_EX_MEM),
		.PC_EX_MEM(PC_EX_MEM),
 
		.WB_en_EX_MEM(WB_en_EX_MEM),

		.result_EX_MEM(result_EX_MEM),
		.MemtoReg_Data_MEM(MemtoReg_Data_MEM),
		.MemtoReg_EX_MEM(MemtoReg_EX_MEM),//无效：ALU算的数->reg；有效：访存->reg
		.RegWrite_EX_MEM(RegWrite_EX_MEM),//无效：不写寄存器；有效：写寄存器

		.nop_MEM_WB(nop_MEM_WB),

		.WB_en_MEM_WB(WB_en_MEM_WB),

		.result_MEM_WB(result_MEM_WB),
		.MemtoReg_Data_MEM_WB(MemtoReg_Data_MEM_WB),
		.MemtoReg_MEM_WB(MemtoReg_MEM_WB),
		.RegWrite_MEM_WB(RegWrite_MEM_WB),

		.PC_MEM_WB(PC_MEM_WB),
		.instruction_MEM_WB(instruction_MEM_WB)
	);

//////////////////////////////////////////////////下面进入WB阶段

	assign RF_wen = WB_en_MEM_WB & !nop_MEM_WB & RegWrite_MEM_WB;

	/*assign RF_wdata_temp =  ({32{Reg_src[0]}} & ALU_out) | 
							({32{Reg_src[1]}} & Shift_out) | 
							({32{Reg_src[2]}} & imm_or_read_data) |
							({32{Reg_src[3]}} & ALU_out) |
							({32{Reg_src[4]}} & ALU_out_next) |
							({32{Reg_src[5]}} & mul_out);*/

	assign RF_wdata = (MemtoReg_MEM_WB) ? MemtoReg_Data_MEM_WB : result_MEM_WB;

	assign RF_waddr = instruction_MEM_WB[11:7];

	//用Regsrc选择PC_write,Result_shift,Result_ALU,imm后面补充16个0,直接寄存器转移rdata1(rs)
	//用MemtoReg选择MEM中数据和上面选择的结果

//////////////////////////////////////////////////最后用寄存器进行存储，并进行指令提交
	reg			nop_retire;
	reg[31:0]	PC_retire;
	reg[31:0]	RF_wdata_retire;
	reg[4:0]	RF_waddr_retire;
	reg			RF_en_rt_retire;
	reg			keep;
	
	always @(posedge clk) 	
	begin
		if(rst) begin
			keep <= 1'b0;
		end
		else if(pipeline)begin
			keep <= 1'b1;
		end
		else begin
			keep <= 1'b0;
		end
    end

	always @(posedge clk) 
	begin
		if(rst) begin
			nop_retire <= 1'b0;
		end
		else if(pipeline)begin
			nop_retire <= nop_MEM_WB;
		end
		else begin
			;
		end
    end

	always @(posedge clk) 
	begin
		if(rst) begin
			PC_retire <= 32'b0;
		end
		else if(pipeline)begin
			PC_retire <= PC_MEM_WB;
		end
		else begin
			;
		end
    end

	always @(posedge clk) 
	begin
		if(rst) begin
			RF_wdata_retire <= 32'b0;
		end
		else if(pipeline)begin
			RF_wdata_retire <= RF_wdata;
		end
		else begin
			;
		end
    end

	always @(posedge clk) 
	begin
		if(rst) begin
			RF_waddr_retire <= 4'b0;
		end
		else if(pipeline)begin
			RF_waddr_retire <= RF_waddr;
		end
		else begin
			;
		end
    end

	always @(posedge clk) 
	begin
		if(rst) begin
			RF_en_rt_retire <= 1'b0;
		end
		else if(pipeline)begin
			RF_en_rt_retire <= RF_wen;
		end
		else begin
			;
		end
    end

	/*always @(posedge clk) 
	begin
		if(rst) begin
			bomb <= 16'b0;
		end
		else begin
			bomb <= bomb + 1'b1;
		end
    end

	assign stop = (bomb == 16'b1111111111111111);

	assign inst_retire[31:0]    = {32{stop}} | ({32{keep}}  & ({32{!nop_retire}} & PC_retire));
	assign inst_retire[63:32]   = {32{stop}} | ({32{keep}}  & ({32{!nop_retire}} & RF_wdata_retire));
	assign inst_retire[68:64]   = {5 {stop}} | ({5 {keep}}  & ({5{!nop_retire}} & RF_waddr_retire));
	assign inst_retire[69] 	    =     stop   | (    keep    & (!nop_retire & RF_en_rt_retire & (RF_waddr_retire != 0)));*/

	assign inst_retire[31:0]  = {32{keep}}  & ({32{!nop_MEM_WB}} & PC_MEM_WB);
	assign inst_retire[63:32] = {32{keep}}  & ({32{!nop_MEM_WB}} & RF_wdata);
	assign inst_retire[68:64] = {5 {keep}}  & ({5 {!nop_MEM_WB}} & RF_waddr);
	assign inst_retire[69] 	  =     keep    & (!nop_MEM_WB & RF_wen & (RF_waddr != 0));
// TODO: Please add your custom CPU code here

endmodule
module control(
	input[31:0]		instruction,
	input[31:0]		PC,
	input[31:0]		Read_data1,
	input[31:0]		Read_data2,

	output			EX_en_ID,
	output			MEM_en_ID,
	output			WB_en_ID,

	output			Branch_ID,
	output[31:0]	read_dataA_ID,
	output[31:0]	read_dataB_ID,
	output      	MemRead_ID,//无效：不读内存；有效：读内存
	output     		MemtoReg_ID,//无效：ALU算的数->reg；有效：访存->reg
	output       	MemWrite_ID,//无效：不写内存；有效：写内存
	output[3:0] 	alu_shifter_mul_imm_ID,//用独热码表示接下来参与运算的数据会进入哪个运算器进行运算
	//0位对应不参加运算，直接放入
	//1位对应参加乘法运算
	//2位对应参加移位运算
	//3位对应参加alu运算
	output	    	RegWrite_ID,//无效：不写寄存器；有效：写寄存器

	output[7:0]		pre_ALUop,
	output[31:0]	imm,
	output[31:0]	store_data_ID,
	output[31:0]	PC_src_data
);
	wire [6:0]		opcode;

	wire [2:0]		ALUop_temp;
	wire [1:0]		ALUsrcA;
	wire [2:0]		ALUsrcB;

	wire [31:0]		imm_extended;

	assign ALUop_temp = instruction[14:12];
	assign opcode = instruction[6:0];

    assign pre_ALUop[0] = (opcode != 0) & (opcode[6] & opcode[5] & !opcode[2]);//B型跳转指令;
    assign pre_ALUop[1] = (opcode != 0) & (opcode[5] & opcode[4] & !opcode[2]);//R寄存器操作指令;
    assign pre_ALUop[2] = (opcode != 0) & (!opcode[5] & opcode[4] & !opcode[2]);//I-type计算指令;
    assign pre_ALUop[3] = (opcode != 0) & (opcode[4] & opcode[2]);//U-Type型指令;
	assign pre_ALUop[4] = (opcode != 0) & (opcode[6] & opcode[5] & opcode[2]);//J-Type指令;
	assign pre_ALUop[5] = (opcode != 0) & (!opcode[6] & opcode[5] & !opcode[4]);//Store-Type指令;
	assign pre_ALUop[6] = (opcode != 0) & (!opcode[6] & !opcode[5] & !opcode[4]);//I-Load指令;
	assign pre_ALUop[7] = 0;
	//preALUop负责对所有指令进行划分

	assign EX_en_ID  = !(pre_ALUop[3] & opcode[5]);
	assign MEM_en_ID = pre_ALUop[5] | pre_ALUop[6];
	assign WB_en_ID  = pre_ALUop[1] | pre_ALUop[2] | pre_ALUop[3] | pre_ALUop[4] | pre_ALUop[6];

	sign_extended sign_extended0(
		.imm_1(instruction[31:12]),
		.imm_2(instruction[11:7]),
		.op_temp(ALUop_temp),
		.opcode(opcode),
		.pre_ALUop(pre_ALUop),

		.imm_extended(imm_extended)
	);

	assign Branch_ID = pre_ALUop[0] | pre_ALUop[4];//这时的Branch 只考虑对beq等分支进行选择，连同后面的judge_branch一起

	assign ALUsrcA[0] = pre_ALUop[3] | pre_ALUop[4];
	assign ALUsrcA[1] = pre_ALUop[0] | pre_ALUop[1] | pre_ALUop[2] | pre_ALUop[5] | pre_ALUop[6];

	assign ALUsrcB[0] = pre_ALUop[4];
	assign ALUsrcB[1] = pre_ALUop[2] | pre_ALUop[3] | pre_ALUop[5] | pre_ALUop[6];
	assign ALUsrcB[2] = pre_ALUop[0] | pre_ALUop[1];

	assign read_dataA_ID =  ({32{ALUsrcA[0]}} & PC) | 
							({32{ALUsrcA[1]}} & Read_data1);

	assign read_dataB_ID =  ({32{ALUsrcB[0]}} & 32'b100) |
			   				({32{ALUsrcB[1]}} & imm_extended) | 
			   				({32{ALUsrcB[2]}} & Read_data2);

	assign MemRead_ID  = pre_ALUop[6];
	assign MemtoReg_ID = pre_ALUop[6];
	assign MemWrite_ID = pre_ALUop[5];

	assign alu_shifter_mul_imm_ID[0] = pre_ALUop[3] & opcode[5];
	assign alu_shifter_mul_imm_ID[1] = pre_ALUop[1] & instruction[25];
	assign alu_shifter_mul_imm_ID[2] = (pre_ALUop[1] | pre_ALUop[2]) & ALUop_temp[0] & !ALUop_temp[1];
	assign alu_shifter_mul_imm_ID[3] = 	!alu_shifter_mul_imm_ID[0] &
										!alu_shifter_mul_imm_ID[1] &
										!alu_shifter_mul_imm_ID[2] ;

	assign RegWrite_ID = pre_ALUop[1] | pre_ALUop[2] | pre_ALUop[3] | pre_ALUop[4] | pre_ALUop[6];

	assign imm = imm_extended;
	assign store_data_ID = Read_data2;
	assign PC_src_data = Read_data1;

endmodule

module ALU_Shifter_control(
	input[2:0] 	ALUop_temp,
	input[7:0] 	pre_ALUop,
	input 	   	judge_1,

	output[2:0] ALUOP,
	output[1:0] shiftOP
);
	wire[2:0] temp_ALUOP_RI_type;
	wire[2:0] temp_ALUOP_SL_JU_type;
	wire[2:0] temp_ALUOP_IDecode_type;
	wire[2:0] temp_ALUOP_B_type;

	wire[2:0] judge_ALUop;

	assign judge_ALUop [0]= (pre_ALUop[1] | pre_ALUop[2]);
	assign judge_ALUop [1]= (pre_ALUop[3] | pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6]);
	assign judge_ALUop [2]= pre_ALUop[0];

	assign temp_ALUOP_RI_type = (ALUop_temp) ? ALUop_temp : {ALUop_temp[2:1],judge_1 & pre_ALUop[1]};
	assign temp_ALUOP_SL_JU_type = 3'b000;
	assign temp_ALUOP_B_type = 3'b001;

	assign ALUOP  = ({3{judge_ALUop[0]}} & temp_ALUOP_RI_type) | 
					({3{judge_ALUop[1]}} & temp_ALUOP_SL_JU_type) |
					({3{judge_ALUop[2]}} & temp_ALUOP_B_type) ; 

	assign shiftOP = {ALUop_temp[2],judge_1};

endmodule

module branch_judge(
	input [31:0] Result,
	input [2:0] Judge_op,

	input Zero,
	input CarryOut,
	input Overflow,

	output branch_judge
);
	wire  temp_branch_judge1;
	wire  temp_branch_judge2;
	wire  temp_branch_judge3;
	
	wire [2:0] judge_select_branch;

	assign judge_select_branch  =   {Judge_op[2] &  Judge_op[1],
									 Judge_op[2] & !Judge_op[1],
									!Judge_op[2] & !Judge_op[1]};

	assign temp_branch_judge1 = (Judge_op[0] ^ Zero);//beq,bne
	assign temp_branch_judge2 = (Judge_op[0] ^ (Overflow ^ Result[31]));//blt,bge
	assign temp_branch_judge3 = (Judge_op[0] ^ CarryOut);//bltu,bgeu

	assign branch_judge  = 	(judge_select_branch[0] & temp_branch_judge1) | 
							(judge_select_branch[1] & temp_branch_judge2) |
							(judge_select_branch[2] & temp_branch_judge3) ;
endmodule
//由ALU算出的result以及overflow等一系列的值，结合beq等指令的操作码，给出在这些情况下是否要实行branch操作
//注意这里仅仅给出branch_judge来判定是否进行分支跳转，计算出的结果不包含跳转地址（？）
//这里其实不用考虑overflow？因为第二个寄存器始终取值为0
//都是有符号数的比较然后跳转？

module sign_extended(
	input [19:0] imm_1,
	input [4:0] imm_2,
	input [2:0] op_temp, 
	input [6:0] opcode,
	input [7:0] pre_ALUop,

	output [31:0] imm_extended
);
	wire [0:0] sign;

	wire [7:0] judge_imm;

	wire [31:0]imm_extended_0;
	wire [31:0]imm_extended_1;
	wire [31:0]imm_extended_2;
	wire [31:0]imm_extended_3;
	wire [31:0]imm_extended_4;
	wire [31:0]imm_extended_5;
	wire [31:0]imm_extended_6;
	wire [31:0]imm_extended_7;

	assign sign = imm_1[19];

	assign judge_imm[0] = pre_ALUop[4] & opcode[3];
	assign judge_imm[1] = pre_ALUop[4] & !opcode[3];
	assign judge_imm[2] = pre_ALUop[0];
	assign judge_imm[3] = pre_ALUop[6];
	assign judge_imm[4] = pre_ALUop[5];
	assign judge_imm[5] = pre_ALUop[2] & !judge_imm[7];
	assign judge_imm[6] = pre_ALUop[3];
	assign judge_imm[7] = pre_ALUop[2] & op_temp[0] & !op_temp[1];

	assign imm_extended_0 = {{11{sign}},imm_1[19],imm_1[7:0],imm_1[8],imm_1[18:9],1'b0};
	assign imm_extended_1 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_2 = {{19{sign}},imm_1[19],imm_2[0],imm_1[18:13],imm_2[4:1],1'b0};
	assign imm_extended_3 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_4 = {{20{sign}},imm_1[19:13],imm_2[4:0]};
	assign imm_extended_5 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_6 = {imm_1[19:0],12'b0};
	assign imm_extended_7 = {27'b0,imm_1[12:8]};

	assign imm_extended  =  ({32{judge_imm[0]}} & imm_extended_0) | 
							({32{judge_imm[1]}} & imm_extended_1) |
							({32{judge_imm[2]}} & imm_extended_2) | 
							({32{judge_imm[3]}} & imm_extended_3) | 
							({32{judge_imm[4]}} & imm_extended_4) | 
							({32{judge_imm[5]}} & imm_extended_5) |
							({32{judge_imm[6]}} & imm_extended_6) |
							({32{judge_imm[7]}} & imm_extended_7) ; 

endmodule

module PC_update(
	input   		clk,
	input [31:0]  	PC_next,
	input			pipeline,
	input 		 	rst,

	output [31:0]  PC
);

	reg [31:0] PC_reg;

	always @(posedge clk) 
	begin
		if(rst) begin
			PC_reg <= -4;
		end
		else if(pipeline)begin
			PC_reg <= PC_next;
		end
		else begin
			;
		end
    end

	assign PC = PC_reg;
endmodule

module IF_IDLE(
	input 			clk,
	input			rst,
	input 			pipeline,

	input[31:0] 	PC_next,

	output[31:0]	PC,
	output			nop_IF_IDLE
);
	reg		nop_IF_IDLE;

	always @(posedge clk) 
	begin
		if(rst) begin
			nop_IF_IDLE <= 1'b1;
		end
		else begin
			nop_IF_IDLE <= 1'b0;
		end
    end

	PC_update PC_updata0(
		.clk(clk),
		.rst(rst),
		.PC_next(PC_next),
		.pipeline(pipeline),

		.PC(PC)
	);
endmodule

module Create_Mask(
	input [31:0] Result,
	input [2:0] mask_op,

	output [3:0] mask
);
	wire [3:0] temp_mask_lb;
	wire [3:0] temp_mask_lh;
	wire [3:0] temp_mask_lw;

	wire [2:0] judge_mask;

	assign temp_mask_lb = {Result[0] & Result[1],!Result[0] & Result[1],Result[0] & !Result[1],!Result[0] & !Result[1]};
	assign temp_mask_lh = {{2{Result[1]}},{2{!Result[1]}}};
	assign temp_mask_lw = 4'b1111;

	assign judge_mask = {!mask_op[0] & mask_op[1],mask_op[0] & !mask_op[1],!mask_op[0] & !mask_op[1]};

	assign mask  =  ({4{judge_mask[0]}} & temp_mask_lb) | 
					({4{judge_mask[1]}} & temp_mask_lh) |
					({4{judge_mask[2]}} & temp_mask_lw) ;

endmodule


module IF_ID_registers(
	input			clk,
	input			rst,
	input 			pipeline,

	input 			nop_IF,//由冒险阻塞单元给出的是否置为nop的信号
	input			nop_IF_IDLE,
	input[31:0] 	instruction_IF,
	input[31:0] 	PC,

	output[31:0]	PC_IF_ID, 
	output 			nop_IF_ID,
	output[31:0] 	instruction_IF_ID
);
	reg			nop_IF_ID;
	reg[31:0]	instruction_IF_ID;
	reg[31:0]	PC_IF_ID;

	always @(posedge clk)begin
		if(rst)begin
			nop_IF_ID <= 1'b1;
		end
		else if(pipeline)begin
			nop_IF_ID <= (nop_IF | nop_IF_IDLE);
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			instruction_IF_ID <= 32'b0;
		end
		else if(pipeline)begin
			instruction_IF_ID <= instruction_IF;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			PC_IF_ID <= 32'b0;
		end
		else if(pipeline)begin
			PC_IF_ID <= PC;
		end
		else begin
			;
		end
	end
endmodule

module ID_EX_registers(
	input			clk,
	input			rst,
	input 			pipeline,

	input 			nop_ID,//由冒险阻塞单元给出的是否置为nop的信号
	input 			nop_IF_ID,//上一级的nop寄存器
	input[31:0] 	PC_IF_ID,
	input[31:0]		instruction_IF_ID,
	input[7:0]		pre_ALUOP_ID,
	input[31:0]		imm_ID,
	input[31:0]		store_data_ID,
	input[31:0]		PC_src_data_ID,
	
 
	input			EX_en_ID,
	input			MEM_en_ID,
	input			WB_en_ID,

	input			Branch_ID,
	input[31:0]		read_dataA_ID,
	input[31:0]		read_dataB_ID,
	input      		MemRead_ID,//无效：不读内存；有效：读内存
	input     		MemtoReg_ID,//无效：ALU算的数->reg；有效：访存->reg
	input       	MemWrite_ID,//无效：不写内存；有效：写内存
	input[3:0] 		alu_shifter_mul_imm_ID,//用独热码表示接下来参与运算的数据会进入哪个运算器进行运算
	//0位对应不参加运算，直接放入
	//1位对应参加乘法运算
	//2位对应参加移位运算
	//3位对应参加alu运算
	input	    	RegWrite_ID,//无效：不写寄存器；有效：写寄存器


	output[31:0]	PC_ID_EX,
	output 			nop_ID_EX,

	output			EX_en_ID_EX,
	output			MEM_en_ID_EX,
	output			WB_en_ID_EX,

	output			Branch_ID_EX,
	output[31:0]	read_dataA_ID_EX,
	output[31:0]	read_dataB_ID_EX,
	output      	MemRead_ID_EX,
	output     		MemtoReg_ID_EX,
	output       	MemWrite_ID_EX,
	output[3:0] 	alu_shifter_mul_imm_ID_EX,
	output	    	RegWrite_ID_EX,

	output[31:0]	instruction_ID_EX,
	output[7:0]		pre_ALUOP_ID_EX,
	output[31:0]	imm_ID_EX,
	output[31:0]	store_data_ID_EX,
	output[31:0]	PC_src_data_ID_EX
);
	reg			nop_ID_EX;
	reg[31:0]	PC_ID_EX;

	reg			EX_en_ID_EX;
	reg			MEM_en_ID_EX;
	reg			WB_en_ID_EX;

	reg			Branch_ID_EX;
	reg[31:0]	read_dataA_ID_EX;
	reg[31:0]	read_dataB_ID_EX;
	reg      	MemRead_ID_EX;//无效：不读内存；有效：读内存
	reg     	MemtoReg_ID_EX;//无效：ALU算的数->reg；有效：访存->reg
	reg       	MemWrite_ID_EX;//无效：不写内存；有效：写内存
	reg[3:0] 	alu_shifter_mul_imm_ID_EX;//用独热码表示接下来参与运算的数据会进入哪个运算器进行运算
	//0位对应不参加运算，直接放入
	//1位对应参加乘法运算
	//2位对应参加移位运算
	//3位对应参加alu运算
	reg	    	RegWrite_ID_EX;//无效：不写寄存器；有效：写寄存器

	reg[31:0]	instruction_ID_EX;
	reg[7:0]	pre_ALUOP_ID_EX;
	reg[31:0]	imm_ID_EX;
	reg[31:0]	store_data_ID_EX;
	reg[31:0]	PC_src_data_ID_EX;

	always @(posedge clk)begin
		if(rst)begin
			nop_ID_EX <= 1'b1;
		end
		else if(pipeline)begin
			nop_ID_EX <= (nop_IF_ID | nop_ID);
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MEM_en_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			MEM_en_ID_EX <= MEM_en_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			WB_en_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			WB_en_ID_EX <= WB_en_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			EX_en_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			EX_en_ID_EX <= EX_en_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			Branch_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			Branch_ID_EX <= Branch_ID;
		end
		else begin
			;
		end
	end
	//无效：无跳转；有效；跳转

	always @(posedge clk)begin
		if(rst)begin
			read_dataA_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			read_dataA_ID_EX <= read_dataA_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			read_dataB_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			read_dataB_ID_EX <= read_dataB_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemRead_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			MemRead_ID_EX <= MemRead_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemtoReg_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			MemtoReg_ID_EX <= MemtoReg_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemWrite_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			MemWrite_ID_EX <= MemWrite_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			alu_shifter_mul_imm_ID_EX <= 4'b0;
		end
		else if(pipeline)begin
			alu_shifter_mul_imm_ID_EX <= alu_shifter_mul_imm_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			RegWrite_ID_EX <= 1'b0;
		end
		else if(pipeline)begin
			RegWrite_ID_EX <= RegWrite_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			PC_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			PC_ID_EX <= PC_IF_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			instruction_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			instruction_ID_EX <= instruction_IF_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			pre_ALUOP_ID_EX <= 8'b0;
		end
		else if(pipeline)begin
			pre_ALUOP_ID_EX <= pre_ALUOP_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			imm_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			imm_ID_EX <= imm_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			store_data_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			store_data_ID_EX <= store_data_ID;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			PC_src_data_ID_EX <= 32'b0;
		end
		else if(pipeline)begin
			PC_src_data_ID_EX <= PC_src_data_ID;
		end
		else begin
			;
		end
	end
endmodule



module EX_MEM_registers(
	input			clk,
	input			rst,
	input 			pipeline,

	input 			nop_ID_EX,//上一级的nop寄存器

	input[31:0]		store_data_ID_EX,
	input[31:0]		instruction_ID_EX,
	input[31:0]		PC_ID_EX,
	
 
	input			MEM_en_ID_EX,
	input			WB_en_ID_EX,

	input[31:0]		result_EX,
	input      		MemRead_ID_EX,//无效：不读内存；有效：读内存
	input     		MemtoReg_ID_EX,//无效：ALU算的数->reg；有效：访存->reg
	input       	MemWrite_ID_EX,//无效：不写内存；有效：写内存
	input	    	RegWrite_ID_EX,//无效：不写寄存器；有效：写寄存器

	output 			nop_EX_MEM,

	output			MEM_en_EX_MEM,
	output			WB_en_EX_MEM,

	output[31:0]	result_EX_MEM,
	output      	MemRead_EX_MEM,
	output     		MemtoReg_EX_MEM,
	output       	MemWrite_EX_MEM,
	output	    	RegWrite_EX_MEM,

	output[31:0]	PC_EX_MEM,
	output[31:0]	store_data_EX_MEM,
	output[31:0]	instruction_EX_MEM
);
	reg			nop_EX_MEM;

	reg			MEM_en_EX_MEM;
	reg			WB_en_EX_MEM;

	reg      	MemRead_EX_MEM;//无效：不读内存；有效：读内存
	reg     	MemtoReg_EX_MEM;//无效：ALU算的数->reg；有效：访存->reg
	reg       	MemWrite_EX_MEM;//无效：不写内存；有效：写内存
	reg	    	RegWrite_EX_MEM;//无效：不写寄存器；有效：写寄存器

	reg[31:0]	PC_EX_MEM;
	reg[31:0]	instruction_EX_MEM;
	reg[31:0]	store_data_EX_MEM;
	reg[31:0]	result_EX_MEM;

	always @(posedge clk)begin
		if(rst)begin
			nop_EX_MEM <= 1'b1;
		end
		else if(pipeline)begin
			nop_EX_MEM <= nop_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MEM_en_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			MEM_en_EX_MEM <= MEM_en_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			WB_en_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			WB_en_EX_MEM <= WB_en_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemRead_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			MemRead_EX_MEM <= MemRead_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemtoReg_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			MemtoReg_EX_MEM <= MemtoReg_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemWrite_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			MemWrite_EX_MEM <= MemWrite_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			RegWrite_EX_MEM <= 1'b0;
		end
		else if(pipeline)begin
			RegWrite_EX_MEM <= RegWrite_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			store_data_EX_MEM <= 32'b0;
		end
		else if(pipeline)begin
			store_data_EX_MEM <= store_data_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			result_EX_MEM <= 32'b0;
		end
		else if(pipeline)begin
			result_EX_MEM <= result_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			instruction_EX_MEM <= 32'b0;
		end
		else if(pipeline)begin
			instruction_EX_MEM <= instruction_ID_EX;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			PC_EX_MEM <= 32'b0;
		end
		else if(pipeline)begin
			PC_EX_MEM <= PC_ID_EX;
		end
		else begin
			;
		end
	end
endmodule

module MEM_WB_registers(
	input			clk,
	input			rst,
	input 			pipeline,

	input 			nop_EX_MEM,//上一级的nop寄存器
	input[31:0]		instruction_EX_MEM,
	input[31:0]		PC_EX_MEM,
 
	input			WB_en_EX_MEM,

	input[31:0]		result_EX_MEM,
	input[31:0]		MemtoReg_Data_MEM,
	input     		MemtoReg_EX_MEM,//无效：ALU算的数->reg；有效：访存->reg
	input	    	RegWrite_EX_MEM,//无效：不写寄存器；有效：写寄存器

	output 			nop_MEM_WB,

	output			WB_en_MEM_WB,

	output[31:0]	result_MEM_WB,
	output[31:0]	MemtoReg_Data_MEM_WB,
	output     		MemtoReg_MEM_WB,
	output	    	RegWrite_MEM_WB,

	output[31:0]	instruction_MEM_WB,
	output[31:0]	PC_MEM_WB
);
	reg			nop_MEM_WB;

	reg			WB_en_MEM_WB;

	reg[31:0]	result_MEM_WB;
	reg[31:0]	MemtoReg_Data_MEM_WB;
	reg     	MemtoReg_MEM_WB;//无效：ALU算的数->reg；有效：访存->reg
	reg	    	RegWrite_MEM_WB;//无效：不写寄存器；有效：写寄存器

	reg[31:0]	instruction_MEM_WB;
	reg[31:0]	PC_MEM_WB;

	always @(posedge clk)begin
		if(rst)begin
			nop_MEM_WB <= 1'b1;
		end
		else if(pipeline)begin
			nop_MEM_WB <= nop_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			WB_en_MEM_WB <= 1'b0;
		end
		else if(pipeline)begin
			WB_en_MEM_WB <= WB_en_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			result_MEM_WB <= 32'b0;
		end
		else if(pipeline)begin
			result_MEM_WB <= result_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemtoReg_Data_MEM_WB <= 32'b0;
		end
		else if(pipeline)begin
			MemtoReg_Data_MEM_WB <= MemtoReg_Data_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			MemtoReg_MEM_WB <= 1'b0;
		end
		else if(pipeline)begin
			MemtoReg_MEM_WB <= MemtoReg_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			RegWrite_MEM_WB <= 1'b0;
		end
		else if(pipeline)begin
			RegWrite_MEM_WB <= RegWrite_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			instruction_MEM_WB <= 32'b0;
		end
		else if(pipeline)begin
			instruction_MEM_WB <= instruction_EX_MEM;
		end
		else begin
			;
		end
	end

	always @(posedge clk)begin
		if(rst)begin
			PC_MEM_WB <= 32'b0;
		end
		else if(pipeline)begin
			PC_MEM_WB <= PC_EX_MEM;
		end
		else begin
			;
		end
	end
endmodule

module pipeline_set(
	input 		nop_IF_IDLE,
	input		nop_IF_ID,
	input		nop_ID_EX,
	input		nop_EX_MEM,
	input		nop_MEM_WB,
	input		MEM_en_EX_MEM,

	input 		pipeline_IF_not_nop,
	input		pipeline_MEM_not_nop,

	output 		pipeline
);
	wire		pipeline_IF;
	wire		pipeline_ID;
	wire		pipeline_EX;
	wire		pipeline_MEM;
	wire		pipeline_WB;

	assign pipeline_IF = nop_IF_IDLE | pipeline_IF_not_nop;
	assign pipeline_ID = 1;
	assign pipeline_EX = 1;
	assign pipeline_MEM = nop_EX_MEM | !MEM_en_EX_MEM | pipeline_MEM_not_nop;
	assign pipeline_WB = 1;

	assign pipeline = pipeline_IF & pipeline_ID & pipeline_EX & pipeline_MEM & pipeline_WB;
endmodule

module Forwarding_unit(
	input[31:0] 	instruction_EX_MEM,
	input[31:0] 	instruction_MEM_WB,
	input[4:0]		RF_waddr_retire,

	input			RegWrite_EX_MEM,
	input			RegWrite_MEM_WB,
	input			RF_en_rt_retire,

	input			nop_EX_MEM,
	input			nop_MEM_WB,
	input			nop_retire,

	input[31:0]		instruction_ID_EX,
	input[7:0]		pre_ALUOP_ID_EX,

	output[3:0]		Forward_select_dataA,
	output[3:0]		Forward_select_dataB,
	output[3:0]		Forward_select_store_data,
	output[3:0]		Forward_select_PC_src_data
);
	wire[3:0]		Forward_select_dataA_temp;
	wire[3:0]		Forward_select_dataB_temp;
	wire[3:0]		Forward_select_store_data_temp;
	wire[3:0]		Forward_select_PC_src_data_temp;



	assign Forward_select_dataA_temp[0] = (RF_waddr_retire == instruction_ID_EX[19:15]) 
											& RF_en_rt_retire & !nop_retire & (RF_waddr_retire != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1] | pre_ALUOP_ID_EX[2] 
											| pre_ALUOP_ID_EX[5] | pre_ALUOP_ID_EX[6]);

	assign Forward_select_dataA_temp[1] = (instruction_MEM_WB[11:7] == instruction_ID_EX[19:15]) 
											& RegWrite_MEM_WB & !nop_MEM_WB & (instruction_MEM_WB[11:7] != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1] | pre_ALUOP_ID_EX[2] 
											| pre_ALUOP_ID_EX[5] | pre_ALUOP_ID_EX[6]);
									
	assign Forward_select_dataA_temp[2] = (instruction_EX_MEM[11:7] == instruction_ID_EX[19:15]) 
											& RegWrite_EX_MEM & !nop_EX_MEM & (instruction_EX_MEM[11:7] != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1] | pre_ALUOP_ID_EX[2] 
											| pre_ALUOP_ID_EX[5] | pre_ALUOP_ID_EX[6]); 

	assign Forward_select_dataA_temp[3] = 	!Forward_select_dataA_temp[2] & 
											!Forward_select_dataA_temp[1] &
											!Forward_select_dataA_temp[0] ;

	assign Forward_select_dataB_temp[0] = (RF_waddr_retire == instruction_ID_EX[24:20]) 
											& RF_en_rt_retire & !nop_retire & (RF_waddr_retire != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1]);

	assign Forward_select_dataB_temp[1] = (instruction_MEM_WB[11:7] == instruction_ID_EX[24:20]) 
											& RegWrite_MEM_WB & !nop_MEM_WB & (instruction_MEM_WB[11:7] != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1]);
									
	assign Forward_select_dataB_temp[2] = (instruction_EX_MEM[11:7] == instruction_ID_EX[24:20]) 
											& RegWrite_EX_MEM & !nop_EX_MEM & (instruction_EX_MEM[11:7] != 0) &
											(pre_ALUOP_ID_EX[0] | pre_ALUOP_ID_EX[1]);

	assign Forward_select_dataB_temp[3]	 = 	!Forward_select_dataB_temp[2] & 
											!Forward_select_dataB_temp[1] &
											!Forward_select_dataB_temp[0] ;								

	assign Forward_select_store_data_temp[0] = 	(RF_waddr_retire == instruction_ID_EX[24:20]) 
												& RF_en_rt_retire & !nop_retire & (RF_waddr_retire != 0) &
												pre_ALUOP_ID_EX[5];

	assign Forward_select_store_data_temp[1] = 	(instruction_MEM_WB[11:7] == instruction_ID_EX[24:20]) 
												& RegWrite_MEM_WB & !nop_MEM_WB & (instruction_MEM_WB[11:7] != 0) &
												pre_ALUOP_ID_EX[5];
									
	assign Forward_select_store_data_temp[2] = 	(instruction_EX_MEM[11:7] == instruction_ID_EX[24:20]) 
												& RegWrite_EX_MEM & !nop_EX_MEM & (instruction_EX_MEM[11:7] != 0) &
												pre_ALUOP_ID_EX[5];  

	assign Forward_select_store_data_temp[3] = 	!Forward_select_store_data_temp[2] & 
												!Forward_select_store_data_temp[1] &
												!Forward_select_store_data_temp[0] ;														 

	assign Forward_select_PC_src_data_temp[0] = (RF_waddr_retire == instruction_ID_EX[19:15]) 
												& RF_en_rt_retire & !nop_retire & (RF_waddr_retire != 0) &
												(pre_ALUOP_ID_EX[4] & !instruction_ID_EX[3]);

	assign Forward_select_PC_src_data_temp[1] = (instruction_MEM_WB[11:7] == instruction_ID_EX[19:15]) 
												& RegWrite_MEM_WB & !nop_MEM_WB & (instruction_MEM_WB[11:7] != 0) &
												(pre_ALUOP_ID_EX[4] & !instruction_ID_EX[3]);

	assign Forward_select_PC_src_data_temp[2] = (instruction_EX_MEM[11:7] == instruction_ID_EX[19:15]) 
												& RegWrite_EX_MEM & !nop_EX_MEM & (instruction_EX_MEM[11:7] != 0) &
												(pre_ALUOP_ID_EX[4] & !instruction_ID_EX[3]);

	assign Forward_select_PC_src_data_temp[3] = !Forward_select_PC_src_data_temp[2] & 
												!Forward_select_PC_src_data_temp[1] &
												!Forward_select_PC_src_data_temp[0] ;

	assign Forward_select_dataA[2]	=	Forward_select_dataA_temp[2];
	assign Forward_select_dataA[1]	=	Forward_select_dataA_temp[1] & !Forward_select_dataA_temp[2];	
	assign Forward_select_dataA[0]	=	Forward_select_dataA_temp[0] & !Forward_select_dataA_temp[1] & !Forward_select_dataA_temp[2];	
	assign Forward_select_dataA[3]	=	Forward_select_dataA_temp[3];	

	assign Forward_select_dataB[2]	=	Forward_select_dataB_temp[2];	
	assign Forward_select_dataB[1]	=	Forward_select_dataB_temp[1] & !Forward_select_dataB_temp[2];	
	assign Forward_select_dataB[0]	=	Forward_select_dataB_temp[0] & !Forward_select_dataB_temp[1] & !Forward_select_dataB_temp[2];	
	assign Forward_select_dataB[3]	=	Forward_select_dataB_temp[3];	

	assign Forward_select_store_data[2]	=	Forward_select_store_data_temp[2];	
	assign Forward_select_store_data[1]	=	Forward_select_store_data_temp[1] & !Forward_select_store_data_temp[2];	
	assign Forward_select_store_data[0]	=	Forward_select_store_data_temp[0] & !Forward_select_store_data_temp[1] & !Forward_select_store_data_temp[2];	
	assign Forward_select_store_data[3]	=	Forward_select_store_data_temp[3];	

	assign Forward_select_PC_src_data[2]	=	Forward_select_PC_src_data_temp[2];	
	assign Forward_select_PC_src_data[1]	=	Forward_select_PC_src_data_temp[1] & !Forward_select_PC_src_data_temp[2];
	assign Forward_select_PC_src_data[0]	=	Forward_select_PC_src_data_temp[0] & !Forward_select_PC_src_data_temp[1] & !Forward_select_PC_src_data_temp[2];	
	assign Forward_select_PC_src_data[3]	=	Forward_select_PC_src_data_temp[3];								
endmodule

module jump_nop_PC(
	input 		exit,
	input[7:0] 	pre_ALUOP_ID,

	input		nop_IF_ID,

	output		nop_IF,
	output		nop_ID,
	output		PC_silent
);
	wire	nop_IF_jump;
	wire	nop_ID_jump;
	wire	nop_IF_silent;

	assign 	nop_IF_jump = exit;
	assign 	nop_ID_jump = exit;
	//由于jump指令需要置的nop

	assign	nop_IF_silent = pre_ALUOP_ID[6] & !nop_IF_ID;
	//由于lw指令需要置的nop

	assign  PC_silent = pre_ALUOP_ID[6] & !nop_IF_ID;

	assign 	nop_IF = nop_IF_jump | nop_IF_silent;
	assign 	nop_ID = nop_ID_jump;
endmodule

module MemtoReg_Data_Read(
	input [3:0] mask,
	input [31:0] Memdata,
	input [2:0] mask_op,

	output [31:0] memtoReg_data
);

	wire sign;
	wire [2:0] judge_mask;

	wire [7:0] temp_memtoReg_data_lb_8bit;
	wire [31:0] temp_memtoReg_data_lb;
	wire [15:0] temp_memtoReg_data_lh_16bit;
	wire [31:0] temp_memtoReg_data_lh;
	wire [31:0] temp_memtoReg_data_lw;

	assign sign = !mask_op[2];

	assign judge_mask = {!mask_op[0] & mask_op[1],mask_op[0] & !mask_op[1],!mask_op[0] & !mask_op[1]};

	assign temp_memtoReg_data_lb_8bit  = ({8{mask[0]}} & Memdata[7:0])   |
										 ({8{mask[1]}} & Memdata[15:8])  | 
										 ({8{mask[2]}} & Memdata[23:16]) | 
										 ({8{mask[3]}} & Memdata[31:24]) ;
	assign temp_memtoReg_data_lb  = {{24{sign & temp_memtoReg_data_lb_8bit[7]}},temp_memtoReg_data_lb_8bit};
	assign temp_memtoReg_data_lh_16bit = ({16{mask[0]}} & Memdata[15:0])  |
										 ({16{mask[2]}} & Memdata[31:16]) ;
	assign temp_memtoReg_data_lh  = {{16{sign & temp_memtoReg_data_lh_16bit[15]}},temp_memtoReg_data_lh_16bit};
	assign temp_memtoReg_data_lw  = Memdata;


	assign memtoReg_data  = ({32{judge_mask[0]}} & temp_memtoReg_data_lb) | 
							({32{judge_mask[1]}} & temp_memtoReg_data_lh) | 
							({32{judge_mask[2]}} & temp_memtoReg_data_lw) ;

endmodule

module Create_Strb(
	input [31:0] Result,
	input [2:0] strb_op,

	output [3:0] strb
);
	wire [3:0] temp_strb_lb;
	wire [3:0] temp_strb_lh;
	wire [3:0] temp_strb_lw;

	wire [2:0] judge_strb;

	assign temp_strb_lb = {Result[0] & Result[1],!Result[0] & Result[1],Result[0] & !Result[1],!Result[0] & !Result[1]};
	assign temp_strb_lh = {{2{Result[1]}},{2{!Result[1]}}};
	assign temp_strb_lw = 4'b1111;

	assign judge_strb = {!strb_op[0] & strb_op[1],strb_op[0] & !strb_op[1],!strb_op[0] & !strb_op[1]};

	assign strb  =  ({4{judge_strb[0]}} & temp_strb_lb) | 
					({4{judge_strb[1]}} & temp_strb_lh) |
					({4{judge_strb[2]}} & temp_strb_lw) ;

endmodule

module RegtoMem_Data_Write(
	input [3:0] strb,
	input [31:0] rt_reg_data,
	input [2:0] strb_op,

	output [31:0] Regtomem_data
);

	wire [31:0] Regtomem_data_sb;
	wire [31:0] Regtomem_data_sh;
	wire [31:0] Regtomem_data_sw;

	wire [2:0]  judge_strb;

	assign judge_strb = {!strb_op[0] & strb_op[1],strb_op[0] & !strb_op[1],!strb_op[0] & !strb_op[1]};
	
	assign Regtomem_data_sb =  {{8{strb[3]}} & rt_reg_data[7:0] , 
								{8{strb[2]}} & rt_reg_data[7:0] ,
				      			{8{strb[1]}} & rt_reg_data[7:0] , 
								{8{strb[0]}} & rt_reg_data[7:0]};
	//进行lb的扩展

	assign Regtomem_data_sh =  {{16{strb[2]}} & rt_reg_data[15:0] , 
								{16{strb[0]}} & rt_reg_data[15:0]};
	//进行lh的扩展

	assign Regtomem_data_sw = rt_reg_data;
	//承接lw的取值

	assign Regtomem_data  = ({32{judge_strb[0]}} & Regtomem_data_sb) | 
							({32{judge_strb[1]}} & Regtomem_data_sh) | 
							({32{judge_strb[2]}} & Regtomem_data_sw) ;

endmodule

module IF_state_change(
	input 		clk,
	input		rst,
	input		pipeline,
	input[31:0]	instruction,

	input 		Inst_Req_Ready,
	input		Inst_Valid,

	input			MEM_en_EX_MEM,
	input			nop_EX_MEM,
	input			MemRead_EX_MEM,

	input  			MemWrite_EX_MEM,

	input[3:0]		MEM_current_state,

	output[31:0]instruction_reg,
	output		pipeline_IF_not_nop,
	output		Inst_Req_Valid,
	output		Inst_Ready
);
	localparam 	Init = 4'b0001,//新加初始状态
				IFetch = 4'b0010,
				IFetch_wait = 4'b0100,
				IFetch_finish = 4'b1000;


	reg [3:0] IF_current_state;
	reg[31:0] instruction_reg;	

	reg [3:0] next_state;

	always @(posedge clk) begin
		if (rst) begin
			IF_current_state <= Init;
		end
		else begin
			IF_current_state <= next_state;
		end
	end

	always @(posedge clk) begin
		if (rst) begin
			instruction_reg <= 32'b0;
		end
		else if(Inst_Ready & Inst_Valid & IF_current_state[2]) begin
			instruction_reg <= instruction;
		end
		else begin
			;
		end
	end
 
	always @(*) begin
		case(IF_current_state)
			IFetch:
				if(Inst_Req_Valid & Inst_Req_Ready)begin
					next_state = IFetch_wait;
				end
				else begin
					next_state = IFetch;
				end
			IFetch_wait:
				if(Inst_Ready & Inst_Valid)begin
					next_state = IFetch_finish;
				end
				else begin
					next_state = IFetch_wait;
				end
			IFetch_finish:
				if(pipeline)begin
					next_state = IFetch;
				end
				else begin
					next_state = IFetch_finish;
				end
			default: 
				next_state = IFetch;
		endcase
	end

	assign	Inst_Req_Valid = ~rst & IF_current_state[1] &  
							(!(MemRead_EX_MEM & !nop_EX_MEM & MEM_en_EX_MEM) | MEM_current_state[3]) &
							(!(MemWrite_EX_MEM & !nop_EX_MEM & MEM_en_EX_MEM) | MEM_current_state[3]) ;
	assign	Inst_Ready = IF_current_state[0] | IF_current_state[2];
	assign 	pipeline_IF_not_nop = IF_current_state[3];
endmodule

module MEM_state_change(
	input 		clk,
	input		rst,
	input		pipeline,
	input[31:0]	Read_data,

	input 		Mem_Req_Ready,
	input		Read_data_Valid,

	input		MemWrite_EX_MEM,
	input		nop_EX_MEM,
	input		MEM_en_EX_MEM,
	input		MemRead_EX_MEM,

	output[3:0]	MEM_current_state,
	output[31:0]MEMdata_reg,
	output		pipeline_MEM_not_nop,
	output		MemWrite,
	output		MemRead,
	output		Read_data_Ready
);
	localparam 	Init = 4'b0001,//新加初始状态
				MEM = 4'b0010,
				MEM_wait = 4'b0100,
				MEM_finish = 4'b1000;


	reg[31:0] MEMdata_reg;
	reg [3:0] MEM_current_state;

	reg [3:0] next_state;

	always @(posedge clk) begin
		if (rst) begin
			MEM_current_state <= Init;
		end
		else begin
			MEM_current_state <= next_state;
		end
	end

	always @(posedge clk) begin
		if (rst) begin
			MEMdata_reg <= 32'b0;
		end
		else if(MEM_current_state[2] & Read_data_Valid & Read_data_Ready)begin
			MEMdata_reg <= Read_data;
		end
		else begin
			;
		end
	end
 
	always @(*) begin
		case(MEM_current_state)
			MEM:
				if(MemRead & Mem_Req_Ready)begin
					next_state = MEM_wait;
				end
				else if(MemWrite & Mem_Req_Ready)begin
					next_state = MEM_finish;
				end
				else begin
					next_state = MEM;
				end
			MEM_wait:
				if(Read_data_Ready & Read_data_Valid)begin
					next_state = MEM_finish;
				end
				else begin
					next_state = MEM_wait;
				end
			MEM_finish:
				if(pipeline)begin
					next_state = MEM;
				end
				else begin
					next_state = MEM_finish;
				end
			default: 
				next_state = MEM;
		endcase
	end

	assign MemWrite = ~rst & MemWrite_EX_MEM & !nop_EX_MEM & MEM_en_EX_MEM & MEM_current_state[1];
	assign MemRead  = ~rst & MemRead_EX_MEM & ~nop_EX_MEM & MEM_en_EX_MEM & MEM_current_state[1];

	assign Read_data_Ready = MEM_current_state[0] | MEM_current_state[2];
	assign pipeline_MEM_not_nop = MEM_current_state[3];
endmodule
