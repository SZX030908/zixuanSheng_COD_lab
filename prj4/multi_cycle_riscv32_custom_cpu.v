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

	wire [31:0]		RF_wdata_temp;
	//寄存器写进数据第一次选择

	wire [31:0]		PC_next;

	wire 			ALUout_write;
	wire 			Shiftout_write;	
	wire 			imm_or_data_reg_write;		
	wire [0:0]		branch_judge;//用在判断beq等指令是否需要跳转
	wire [2:0]		PC_src;
	wire [0:0] 		IR_write;
	wire [0:0] 		PC_write;
	wire [7:0]		pre_ALUOP;
	wire [5:0]     	Reg_src;
	wire [0:0]      Branch;
	//MemRead,MemWrite信号在IO中定义
	wire [0:0]      MemtoReg;
	wire [2:0]      ALUsrcB;
	wire [2:0]      ALUsrcA;
	//RegWrite即为RF_wen
	wire [0:0]      shift_src;
	//以上是由control给出的信号（第一级译码）


	wire [2:0] 		ALUOP;//输出ALUOP
	wire [1:0] 		shiftOP;//输出shiftOP
	//第二级译码


	wire [31:0] 	Read_data1 ; 
	wire [31:0] 	Read_data2;
	//从寄存器中读出的数据
	wire [31:0] 	imm_extended;//得到进行扩展之后的{32位立即数
	wire [31:0] 	imm_extended_jmp;//得到进行扩展之后的{32位立即数,左移两位，准备进行跳转指令
	wire [31:0]		imm_adder;
	//有关立即数的变化

	wire [31:0] 	ALU_data1;
	wire [31:0] 	ALU_data2;//用在ALU中进行计算
	wire [4:0] 		shift_data1;
	wire [31:0] 	shift_data2;//用在shifter中进行计算
	//用alu_src与shift_src在上面进行选择

	wire [31:0] 	Result_shift;//用在shifter中进行计算
	wire [31:0] 	Result_ALU;//用在ALU中进行计算
	wire Zero;
	wire CarryOut;
	wire Overflow;

	wire [31:0] 	address_temp;//PC+4与{32位立即数的相加进行选择
	wire [31:0] 	address_reg;//承接寄存器的跳转
	wire [31:0] 	address_jumplong;//承接26位长跳转
	//计算地址跳转，然后用pc_src进行选择

	wire [3:0] 		Mask;//用在ALU中进行计算得到从内存中取的地址
	//以上两者均需要ALU的结果
	wire [31:0] 	MemtoReg_Data;//通过address中取出的数，制作出应该放进rt寄存器种的数
	wire [7:0] 		current_state;


	wire [31:0] 	PC_not_beq;


	reg  [31:0] 	instruction_reg;//多周期存放指令寄存器
	reg  [31:0] 	read_dataA_reg;//多周期存放operand1寄存器
	reg  [31:0] 	read_dataB_reg;//多周期存放operand2寄存器
	reg  [31:0] 	ALU_out;//存放ALU结果寄存器
	reg  [31:0] 	ALU_out_next;//存放ALUout下一轮，以保证数据不被冲掉的寄存器
	//由于现在ALU_out本身就不变，因此不用对何时写next进行控制
	reg  [31:0] 	Shift_out;//存放shift结果寄存器
	reg  [31:0] 	mem_data_reg;//存放读内存结果寄存器
	//不用进行控制，一旦得到内存中数据，下一个状态就是写回
	reg  [31:0] 	imm_or_read_data;//存放直接立即数或寄存器中的数据（不进行操作）

	reg [31:0]		mul_out;
	wire			mul_out_write;
	wire[63:0] 		mul_result;	
	


	reg [31:0] cycle_cnt;//总周期数
	reg [31:0] execute_cycle;//执行指令数
	reg [31:0] mem_cycle;//访存指令数
	reg [31:0] mem_delay;//访存延时
	reg [31:0] branch_jump;//访存指令数
	//性能计数器

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


	always @ (posedge clk)
	begin
			if(rst) begin
					execute_cycle <= 32'b0;
			end
			else if(!rst & current_state[1]) begin
					execute_cycle <= execute_cycle + 32'b1;
			end
			else begin
				;
			end
	end
	assign cpu_perf_cnt_1 = execute_cycle;


	always @ (posedge clk)
	begin
			if(rst) begin
					mem_cycle <= 32'b0;
			end
			else if(!rst & (pre_ALUOP[5] | (pre_ALUOP[6]) & current_state[2])) begin
					mem_cycle <= mem_cycle + 32'b1;
			end
			else begin
				;
			end
	end
	assign cpu_perf_cnt_2 = mem_cycle;


	always @ (posedge clk)
	begin
			if(rst) begin
					mem_delay <= 32'b0;
			end
			else if(!rst & (MemRead | MemWrite | Inst_Req_Valid | Inst_Ready | Read_data_Ready)) begin
					mem_delay <= mem_delay + 32'b1;
			end
			else begin
				;
			end
	end
	assign cpu_perf_cnt_3 = mem_delay;


	always @ (posedge clk)
	begin
			if(rst) begin
					branch_jump <= 32'b0;
			end
			else if(!rst & (PC_src[1] | PC_src[2]) & current_state[2]) begin
					branch_jump <= branch_jump + 32'b1;
			end
			else begin
				;
			end
	end
	assign cpu_perf_cnt_4 = branch_jump;


	PC_update pc_update(
		.clk(clk),
		.PC_next(PC_next),
		.PC_write(PC_write),
		.rst(rst),

		.PC(PC)
	);

	state_change state_change0(
	 	.instruction(instruction_reg),
	 	.clk(clk),
	 	.pre_ALUop(pre_ALUOP),
		.rst(rst),

		.Inst_Req_Ready(Inst_Req_Ready),
		.Inst_Valid(Inst_Valid),
		.Read_data_Valid(Read_data_Valid),
		.Mem_Req_Ready(Mem_Req_Ready),

		.current_state(current_state)
	);

	always @(posedge clk) 
	begin///////////////////////还需要IRwrite
		if(rst) begin
			instruction_reg <= 0;
		end
		else if(!rst & IR_write) begin
			instruction_reg <= Instruction;
		end
		else begin
			;
		end
    end

	reg_file reg0 (
		.clk(clk),
		.waddr(RF_waddr),
		.raddr1(instruction_reg[19:15]),
		.raddr2(instruction_reg[24:20]),
		.wen(RF_wen),
		.wdata(RF_wdata),
		.rdata1(Read_data1),
		.rdata2(Read_data2)
	);

	always @(posedge clk) begin///////////////////////还需要IRwrite
		read_dataA_reg <= Read_data1;
		read_dataB_reg <= Read_data2;
    end

	control control0(
		.Result(Result_ALU),
		.instruction(instruction_reg),
		.current_state(current_state),

		.Zero(Zero),
		.CarryOut(CarryOut),
		.Overflow(Overflow),
		
		.Inst_Valid(Inst_Valid),

		.Inst_Req_Valid(Inst_Req_Valid),
		.Inst_Ready(Inst_Ready),
		.Read_data_Ready(Read_data_Ready),

		.ALUout_write(ALUout_write),
		.Shiftout_write(Shiftout_write),	
		.imm_or_data_reg_write(imm_or_data_reg_write),
		.branch_judge(branch_judge),
		.PCsrc(PC_src),
		.IRwrite(IR_write), 
		.PCwrite(PC_write),
		.ALUsrcA(ALUsrcA),
		.ALUsrcB(ALUsrcB),
        .pre_ALUop(pre_ALUOP),
		.Reg_src(Reg_src),
		.Branch(Branch),
		.MemRead(MemRead),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.RegWrite(RF_wen),
		.shift_src(shift_src),

		.mul_out_write(mul_out_write)
	);

	sign_extended sign0(
		.imm_1(instruction_reg[31:12]),
		.imm_2(instruction_reg[11:7]),
		.op_temp(instruction_reg[14:12]),
		.opcode(instruction_reg[6:0]),
		.pre_ALUop(pre_ALUOP),

		.imm_extended(imm_extended)
	);

	ALU_control alu_control(
		.ALUop_temp(instruction_reg[14:12]),
		.current_state(current_state),
		.pre_ALUop(pre_ALUOP),
		.judge_1(instruction_reg[30]),

		.ALUOP(ALUOP),
		.shiftOP(shiftOP)
	);

	//assign imm_extended_jmp = {imm_extended[29:0],2'b0};//左移两位，既可能是分支跳转的要求，也可能是长跳转的要求
	//assign imm_adder = (pre_ALUOP[3]) ? imm_extended_jmp : imm_extended;//是直接与长跳转（左移4位），还是直接扩展后的立即数
	assign ALU_data1 =  ({32{ALUsrcA[1]}} & read_dataA_reg) | 
						({32{ALUsrcA[0]}} & PC) | 
						({32{ALUsrcA[2]}} & Read_data1) ;

	assign ALU_data2 =  ({32{ALUsrcB[0]}} & read_dataB_reg) |
			   			({32{ALUsrcB[1]}} & imm_extended) | 
			   			({32{ALUsrcB[2]}} & 32'b100);
	
	assign shift_data1 = (shift_src) ? instruction_reg[24:20] : Read_data2[4:0];
	assign shift_data2 = Read_data1;

	shifter shifter0(
		.A(shift_data2),
		.B(shift_data1),
		.Shiftop(shiftOP),

		.Result(Result_shift)
	);

	ALU alu(
		.A(ALU_data1),
		.B(ALU_data2),
		.ALUop(ALUOP),

		.Overflow(Overflow),
		.CarryOut(CarryOut),
		.Zero(Zero),
		.Result(Result_ALU)
	);

	assign mul_result = read_dataA_reg * read_dataB_reg;

	always @(posedge clk) begin
		if(mul_out_write)begin
			mul_out <= mul_result[31:0];
		end
		else begin
			;
		end
    end

	always @(posedge clk) begin
		if(ALUout_write)begin
			ALU_out <= Result_ALU;
		end
		else begin
			;
		end
    end

	always @(posedge clk) begin
		if(Shiftout_write)begin
			Shift_out <= Result_shift;
		end
		else begin
			;
		end
    end

	always @(posedge clk) begin
		if(imm_or_data_reg_write & pre_ALUOP[3])begin
			imm_or_read_data <= imm_extended;
		end
		else begin
			;
		end
    end
	//这里只是针对一些大的情况把相应的数据写进imm_or_read_data寄存器中，用不用这个寄存器中的数据还不好说
	//最迟在写回阶段前把数据写进这个寄存器中，然后在取指阶段时通过选择，看是否把这个数据写进寄存器中

	assign PC_not_beq = PC + 4;
	assign PC_next = ({32{PC_src[0]}} & PC_not_beq)
					|({32{PC_src[1]}} & ALU_out) 
					|({32{PC_src[2]}} & ALU_out);




	assign Address = {ALU_out[31:2],2'b00};
	Create_Mask create_mask(
		.Result(ALU_out_next),
		.mask_op(instruction_reg[14:12]),

		.mask(Mask)
	);

	MemtoReg_Data_Read memtoreg_data_read(
		.mask(Mask),
		.Memdata(mem_data_reg),
		.mask_op(instruction_reg[14:12]),

		.memtoReg_data(MemtoReg_Data)
	);

	Create_Strb create_strb(
		.Result(ALU_out),
		.strb_op(instruction_reg[14:12]),

		.strb(Write_strb)
	);

	RegtoMem_Data_Write regtomem_data_write(
		.strb(Write_strb),
		.rt_reg_data(Read_data2),
	 	.strb_op(instruction_reg[14:12]),

 		.Regtomem_data(Write_data)
	);



	always @(posedge clk) begin
		mem_data_reg <= Read_data;
    end

	always @(posedge clk) begin
		ALU_out_next <= ALU_out;
    end
	//在内存访问阶段，通过已有的ALU_out来给出是否该写入，写入的数据，或者把读出的数据放入mem_data_reg寄存器中
	//相对应的，ALU_out放入ALU_out_next中，并且把抹零后剩下的信息放到新的寄存器里



	assign RF_wdata_temp =  ({32{Reg_src[0]}} & ALU_out) | 
							({32{Reg_src[1]}} & Shift_out) | 
							({32{Reg_src[2]}} & imm_or_read_data) |
							({32{Reg_src[3]}} & ALU_out) |
							({32{Reg_src[4]}} & ALU_out_next) |
							({32{Reg_src[5]}} & mul_out);

	assign RF_wdata = (MemtoReg) ? MemtoReg_Data : RF_wdata_temp;

	assign RF_waddr = instruction_reg[11:7];

	//用Regsrc选择PC_write,Result_shift,Result_ALU,imm后面补充16个0,直接寄存器转移rdata1(rs)
	//用MemtoReg选择MEM中数据和上面选择的结果

// TODO: Please add your custom CPU code here

endmodule
module control(
	input[31:0]		Result,
	input[31:0]		instruction,
	input [7:0]		current_state,

	input 			Zero,
	input 			CarryOut,
	input 			Overflow,

	input         	Inst_Valid,

	output			Inst_Req_Valid,
	output			Inst_Ready,
	output			Read_data_Ready,

	output 		 	ALUout_write,
	output 		 	Shiftout_write,
	output 		 	imm_or_data_reg_write,
	output		 	branch_judge,
	output[2:0]	 	PCsrc,
	output  		IRwrite, 
	output[0:0]	 	PCwrite,

    output[7:0]      pre_ALUop,
	output[5:0]      Reg_src,//分别指代（PC+8），SAL,imm，直接的寄存器，ALU_out,ALU_out_next
	//output[2:0]      RegDst,//RegDst[0]：写到rd寄存器；RegDst[1]：写到rt寄存器;RegDst[2]:31号寄存器
	//现在信号不用给出，因为rd始终在指令中
	output[0:0]      Branch,//无效：无跳转；有效；跳转
	output[0:0]      MemRead,//无效：不读内存；有效：读内存
	output[0:0]      MemtoReg,//无效：ALU算的数->reg；有效：访存->reg
	output[0:0]      MemWrite,//无效：不写内存；有效：写内存
	output[2:0]      ALUsrcB,//无效：立即数不参与运算；有效：立即数参与运算
	output[2:0]      ALUsrcA,//无效：立即数不参与运算；有效：立即数参与运算
	output[0:0]      RegWrite,//无效：不写寄存器；有效：写寄存器
	output[0:0]      shift_src,//无效：立即数不参与运算；有效：立即数参与运算

	output			 mul_out_write
);
	wire [6:0]		opcode;

	wire [2:0]		ALUop_temp;
	wire [1:0]      branch_src;

	assign ALUop_temp = instruction[14:12];
	assign opcode = instruction[6:0];

    assign pre_ALUop[0] = (opcode[6] & opcode[5] & !opcode[2]);//B型跳转指令;
    assign pre_ALUop[1] = (opcode[5] & opcode[4] & !opcode[2]);//R寄存器操作指令;
    assign pre_ALUop[2] = (!opcode[5] & opcode[4] & !opcode[2]);//I-type计算指令;
    assign pre_ALUop[3] = (opcode[4] & opcode[2]);//U-Type型指令;
	assign pre_ALUop[4] = (opcode[6] & opcode[5] & opcode[2]);//J-Type指令;
	assign pre_ALUop[5] = (!opcode[6] & opcode[5] & !opcode[4]);//Store-Type指令;
	assign pre_ALUop[6] = (!opcode[6] & !opcode[5] & !opcode[4]);//I-Load指令;
	assign pre_ALUop[7] = 0;
	//preALUop负责对所有指令进行划分

	assign RegWrite = (pre_ALUop[1] | pre_ALUop[2] | pre_ALUop[3] | pre_ALUop[4] | pre_ALUop[6]) & current_state[4];
	//RegWrite负责控制是否写寄存器

	assign Branch = pre_ALUop[0];//这时的Branch 只考虑对beq等分支进行选择，连同后面的judge_branch一起

	assign branch_src[1] = pre_ALUop[4];//使用j型进行跳转
	assign branch_src[0] = !branch_src[1];//一般的PC+4与beq等一系列指令的综合(即选择由Branch给出的跳转选择)


	assign MemRead  = pre_ALUop[6]&current_state[5];
	assign MemtoReg = pre_ALUop[6];
	assign MemWrite = pre_ALUop[5]&current_state[3];
	//写内存，读内存，写寄存器的数据是否为都内存的数据

	
	assign	Inst_Req_Valid = current_state[0];
	assign	Inst_Ready = current_state[6] | (!current_state);
	assign	Read_data_Ready = current_state[7] | (!current_state);
	//关于真实内存的信号

	assign ALUsrcB[2] = pre_ALUop[4] & current_state[2];//执行阶段，需要算jal;
	assign ALUsrcB[1] = ((pre_ALUop[2] | pre_ALUop[6] | pre_ALUop[5] | pre_ALUop[3]) & current_state[2]) |
						((pre_ALUop[0] | pre_ALUop[4]) & current_state[1]);
	//执行阶段，计算立即数,移位或者不移位
	assign ALUsrcB[0] = !ALUsrcB[1] & !ALUsrcB[2] & current_state[2];//执行阶段，计算read_data2
	//imm_extended,4,0,移位(或不移位的)的立即数(负责进行取指令的),寄存器中的数据

	assign ALUsrcA[0] = (((pre_ALUop[4] & opcode[3]) | pre_ALUop[0]) & current_state[1]) |
						((pre_ALUop[4] | pre_ALUop[3]) & current_state[2]);
	assign ALUsrcA[1] = !ALUsrcA[0] & !ALUsrcA[2];
	assign ALUsrcA[2] = pre_ALUop[4] & !opcode[3] & current_state[1];
	//PC地址，Read_data1

	assign shift_src = !opcode[5];
	//移位器的选择，寄存器中的后五位或是shamt

	assign Reg_src[0] = (pre_ALUop[3] & !opcode[5]) | pre_ALUop[4] ? 1 : 0;
	//对应Auipc,jal,jalr等指令，需要pc的参与
	assign Reg_src[1] = ((pre_ALUop[1] & !instruction[25]) | pre_ALUop[2]) & (ALUop_temp[1:0] == 2'b01)? 1 : 0;//选择数据时对应移位后的数据
	assign Reg_src[2] = (pre_ALUop[3] & opcode[5]) ? 1 : 0;//选择数据时，直接选择立即数后面添0放进寄存器
	assign Reg_src[3] = ((pre_ALUop[1] & !instruction[25]) | pre_ALUop[2]) & !(ALUop_temp[1:0] == 2'b01) ? 1 : 0;
				//选择ALU_out计算出的结果(如果是立即数，也需要即刻写回),为jal或jalr指令
				//凡是需要立马进入写回状态的，都需要在此刻立马写回
	assign Reg_src[4] = !Reg_src[0] & !Reg_src[1] & !Reg_src[2] & !Reg_src[3] & !Reg_src[5];//其他情况均选择ALU_out_next计算出的结果
	assign Reg_src[5] = pre_ALUop[1] & instruction[25];
	//这里只是进行第一次的选择，后面还会用MEM_Read来选择是否是从内存中读的数据
	//关于寄存器数据的选择
	

	assign PCsrc[2] = branch_src[1] & current_state[2];
	assign PCsrc[1] = branch_src[0] & Branch & branch_judge & current_state[2];
	assign PCsrc[0] = !PCsrc[2] & !PCsrc[1];
	//0位对应PC+4，来自其他加法器，
	//1位对应PC+imm跳转
	//2位对应的是j跳转


	branch_judge branch_judge0(
		.Result(Result),
		.Judge_op(ALUop_temp),

		.Zero(Zero),
		.CarryOut(CarryOut),
		.Overflow(Overflow),

		.branch_judge(branch_judge)
	);

	assign PCwrite = current_state[2];
	//何时可以重置PC
	assign IRwrite = current_state[6];
	assign mul_out_write = current_state[2];
	//何时可以写乘法寄存器
	assign ALUout_write = current_state[1] | current_state[2];
	assign Shiftout_write = current_state[2];
	assign imm_or_data_reg_write = current_state[2];

	//assign select_mul = instruction[25];

endmodule

module ALU_control(
	input[2:0] ALUop_temp,
	input[7:0] current_state,
	input[7:0] pre_ALUop,
	input 	judge_1,

	output[2:0] ALUOP,
	output[1:0] shiftOP
);
	wire[2:0] temp_ALUOP_RI_type;
	wire[2:0] temp_ALUOP_SL_JU_type;
	wire[2:0] temp_ALUOP_IDecode_type;
	wire[2:0] temp_ALUOP_B_type;

	wire[3:0] judge_ALUop;

	assign judge_ALUop [0]= (pre_ALUop[1] | pre_ALUop[2]) & current_state[2];
	assign judge_ALUop [1]= (pre_ALUop[3] | pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6]) & current_state[2];
	assign judge_ALUop [2]= current_state[1];
	assign judge_ALUop [3]= pre_ALUop[0] & current_state[2];

	assign temp_ALUOP_RI_type = (ALUop_temp) ? ALUop_temp : {ALUop_temp[2:1],judge_1 & pre_ALUop[1]};
	assign temp_ALUOP_SL_JU_type = 3'b000;
	assign temp_ALUOP_IDecode_type = 3'b000;
	assign temp_ALUOP_B_type = 3'b001;

	assign ALUOP  = ({3{judge_ALUop[0]}} & temp_ALUOP_RI_type) | 
					({3{judge_ALUop[1]}} & temp_ALUOP_SL_JU_type) |
					({3{judge_ALUop[2]}} & temp_ALUOP_IDecode_type) |
					({3{judge_ALUop[3]}} & temp_ALUOP_B_type) ; 

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

	wire [6:0] judge_imm;

	wire [31:0]imm_extended_0;
	wire [31:0]imm_extended_1;
	wire [31:0]imm_extended_2;
	wire [31:0]imm_extended_3;
	wire [31:0]imm_extended_4;
	wire [31:0]imm_extended_5;
	wire [31:0]imm_extended_6;

	assign sign = !(pre_ALUop[2] & op_temp) & imm_1[19];

	assign judge_imm[0] = pre_ALUop[4] & opcode[3];
	assign judge_imm[1] = pre_ALUop[4] & !opcode[3];
	assign judge_imm[2] = pre_ALUop[0];
	assign judge_imm[3] = pre_ALUop[6];
	assign judge_imm[4] = pre_ALUop[5];
	assign judge_imm[5] = pre_ALUop[2];
	assign judge_imm[6] = pre_ALUop[3];

	assign imm_extended_0 = {{11{sign}},imm_1[19],imm_1[7:0],imm_1[8],imm_1[18:9],1'b0};
	assign imm_extended_1 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_2 = {{19{sign}},imm_1[19],imm_2[0],imm_1[18:13],imm_2[4:1],1'b0};
	assign imm_extended_3 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_4 = {{20{sign}},imm_1[19:13],imm_2[4:0]};
	assign imm_extended_5 = {{20{sign}},imm_1[19:8]};
	assign imm_extended_6 = {imm_1[19:0],12'b0};

	assign imm_extended  =  ({32{judge_imm[0]}} & imm_extended_0) | 
							({32{judge_imm[1]}} & imm_extended_1) |
							({32{judge_imm[2]}} & imm_extended_2) | 
							({32{judge_imm[3]}} & imm_extended_3) | 
							({32{judge_imm[4]}} & imm_extended_4) | 
							({32{judge_imm[5]}} & imm_extended_5) |
							({32{judge_imm[6]}} & imm_extended_6) ; 

endmodule

module PC_update(
	input   clk,
	input  [31:0]  PC_next,
	input [0:0] PC_write,
	input [0:0] rst,

	output [31:0]  PC
);

	reg [31:0] PC_reg;

	always @(posedge clk) 
	begin
		if(rst) begin
			PC_reg <= 0;
		end
		else if(PC_write)begin
			PC_reg <= PC_next;
		end
		else begin
			;
		end
    end

	assign PC = PC_reg;
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

module state_change(
	input [31:0] 	instruction,
	input 		clk,
	input		rst,
	input [7:0] 	pre_ALUop,

	input 		Inst_Req_Ready,
	input		Inst_Valid,
	input		Mem_Req_Ready,
	input		Read_data_Valid,

	output [7:0]    current_state
);
	localparam 	Init = 8'b00000000,//新加初始状态
			IFetch = 8'b00000001,//没变,取指
			IWait = 8'b01000000,//第七位，读指令等待
			IDecode = 8'b00000010,//没变，译码
			Execute = 8'b00000100,//没变，执行
			Store = 8'b00001000,//没变，内存写
			Load = 8'b00100000,//第六位，为读内存
			Read_data_wait = 8'b10000000,//第八位，读数据等待
			Write_back = 8'b00010000;//没变，写回

	reg [7:0] current_state;

	reg [7:0] next_state;

	always @(posedge clk) begin
		if (rst) begin
			current_state <= Init;
		end
		else begin
			current_state <= next_state;
		end
	end
 
	always @(*) begin
		case(current_state)
			IFetch:
				if(Inst_Req_Ready)begin
					next_state = IWait;
				end
				else begin
					next_state = IFetch;
				end
			IWait:
				if(Inst_Valid)begin
					next_state = IDecode;
				end
				else begin
					next_state = IWait;
				end
			IDecode:
				if(instruction[31:0] == 32'b0)begin
					next_state = IFetch;
				end
				else begin
					next_state = Execute;
				end
			Execute:
				if( pre_ALUop[0]) begin
					next_state = IFetch;
				end
				else if(pre_ALUop[1] | pre_ALUop[2] | pre_ALUop[3] | pre_ALUop[4]) begin
					next_state = Write_back;
				end
				else if(pre_ALUop[5])begin
					next_state = Store;
				end
				else begin
					next_state = Load;
				end
			Store:
				if(Mem_Req_Ready) begin
					next_state = IFetch;
				end
				else begin
					next_state = Store;
				end
			Load:
				if(Mem_Req_Ready) begin
					next_state = Read_data_wait;
				end
				else begin
					next_state = Load;
				end
			Read_data_wait:
				if(Read_data_Valid) begin
					next_state = Write_back;
				end
				else begin
					next_state = Read_data_wait;
				end
			Write_back: 
				next_state = IFetch;
			default: 
				next_state = IFetch;
		endcase
	end

endmodule
//状态机只负责进行状态转变以及下一状态的判断
