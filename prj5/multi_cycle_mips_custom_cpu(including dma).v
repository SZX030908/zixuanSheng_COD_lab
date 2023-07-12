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
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
 	wire [69:0] inst_retire;



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
	wire [5:0]		PC_src;
	wire [0:0] 		IR_write;
	wire [0:0] 		PC_write;
	wire [7:0]		pre_ALUOP;
	wire [5:0]     	Reg_src;
	wire [2:0]      RegDst;
	wire [0:0]      Branch;
	//MemRead,MemWrite信号在IO中定义
	wire [0:0]      MemtoReg;
	wire [4:0]      ALUsrcB;
	wire [1:0]      ALUsrcA;
	//RegWrite即为RF_wen
	wire [0:0]      shift_src;
	//以上是由control给出的信号（第一级译码）


	wire [2:0] 		ALUOP;//输出ALUOP
	wire [1:0] 		shiftOP;//输出shiftOP
	//第二级译码


	wire [31:0] 	Read_data1;
	wire [31:0] 	Read_data2;
	//从寄存器中读出的数据
	wire [31:0] 	imm_extended;//得到进行扩展之后的32位立即数
	wire [31:0] 	imm_extended_jmp;//得到进行扩展之后的32位立即数,左移两位，准备进行跳转指令
	wire [31:0]		imm_adder;
	//有关立即数的变化

	wire [31:0] 	ALU_data1;
	wire [31:0] 	ALU_data2;//用在ALU中进行计算
	wire [4:0] 		shift_data1;
	wire [31:0] 	shift_data2;//用在shifter中进行计算
	//用alu_src与shift_src在上面进行选择

	wire [31:0] 	Result_shift;//用在shifter中进行计算
	wire [31:0] 	Result_ALU;//用在ALU中进行计算

	wire [31:0] 	address_temp;//PC+4与32位立即数的相加进行选择
	wire [31:0] 	address_reg;//承接寄存器的跳转
	wire [31:0] 	address_jumplong;//承接26位长跳转
	//计算地址跳转，然后用pc_src进行选择

	wire [3:0] 		Mask;//用在ALU中进行计算得到从内存中取的地址
	//以上两者均需要ALU的结果
	wire [31:0] 	MemtoReg_Data;//通过address中取出的数，制作出应该放进rt寄存器种的数
	wire [8:0] 		current_state;




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

	reg [31:0] cycle_cnt;//总周期数
	reg [31:0] execute_cycle;//执行指令数
	reg [31:0] mem_cycle;//访存指令数
	reg [31:0] mem_delay;//访存延时
	reg [31:0] branch_jump;//访存指令数
	//性能计数器

	wire			intr_open_temp;
	wire			intr_valid;
	reg  [31:0] 	EPC;//保存程序执行现场
	reg  			intr_reg;//intr寄存器

	assign intr_valid = intr & intr_reg;
	

	always @ (posedge clk)
	begin
			if(rst) begin
				intr_reg <= 1'b1;
			end
			else if(intr_open_temp)begin
				intr_reg <= 1'b1;
			end
			else if(current_state[8])begin
				intr_reg <= 1'b0;
			end
			else begin
				;
			end
	end 

	always @ (posedge clk)
	begin
			if(rst) begin
				EPC <= 32'b0;
			end
			else if(current_state[8])begin
				EPC <= PC;
			end
			else begin
				;
			end
	end

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
			else if(!rst & (PC_src[1] | PC_src[3] | PC_src[4]) & current_state[2]) begin
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
		.current_state(current_state),

		.PC(PC)
	);

	state_change state_change0(
	 	.instruction(instruction_reg),
	 	.clk(clk),
	 	.pre_ALUop(pre_ALUOP),
		.rst(rst),
		.intr_valid(intr_valid),

		.Inst_Req_Valid(Inst_Req_Valid),
		.Inst_Req_Ready(Inst_Req_Ready),
		.Inst_Valid(Inst_Valid),
		.Read_data_Valid(Read_data_Valid),
		.Mem_Req_Ready(Mem_Req_Ready),

		.current_state(current_state)
	);

	always @(posedge clk) begin///////////////////////还需要IRwrite
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
		.raddr1(instruction_reg[25:21]),
		.raddr2(instruction_reg[20:16]),
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
		.rt(Read_data2),
		.current_state(current_state),
		
		.intr_valid(intr_valid),
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
		.RegDst(RegDst),
		.Branch(Branch),
		.MemRead(MemRead),
		.MemtoReg(MemtoReg),
		.MemWrite(MemWrite),
		.RegWrite(RF_wen),
		.shift_src(shift_src),

		.intr_open_temp(intr_open_temp)
	);

	sign_extended sign0(
		.imm(instruction_reg[15:0]),
		.opcode(instruction_reg[31:26]),
		.pre_ALUop(pre_ALUOP),

		.imm_extended(imm_extended)
	);

	ALU_control alu_control(
		.opcode(instruction_reg[31:26]),
		.pre_ALUop(pre_ALUOP),
		.func(instruction_reg[5:0]),
		.current_state(current_state),

		.ALUOP(ALUOP)
	);

	shift_control shifter_control (
		.func(instruction_reg[5:0]),

		.shiftop(shiftOP)
	);

	assign imm_extended_jmp = {imm_extended[29:0],2'b0};//左移两位，既可能是分支跳转的要求，也可能是长跳转的要求
	assign imm_adder = (pre_ALUOP[3]) ? imm_extended_jmp : imm_extended;//是直接与长跳转（左移4位），还是直接扩展后的立即数
	assign ALU_data1 = ({32{ALUsrcA[1]}} & read_dataA_reg) | ({32{ALUsrcA[0]}} & PC);
	assign ALU_data2 = ({32{ALUsrcB[0]}} & read_dataB_reg) |
			   ({32{ALUsrcB[1]}} & imm_adder) | 
			   ({32{ALUsrcB[2]}} & 32'b0) |
			   ({32{ALUsrcB[3]}} & 32'b100) |
			   ({32{ALUsrcB[4]}} & imm_extended_jmp);
	
	assign shift_data1 = (shift_src) ? instruction_reg[10:6] : Read_data1[4:0];
	assign shift_data2 = Read_data2;

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
		if(imm_or_data_reg_write & pre_ALUOP[4])begin
			imm_or_read_data <= {imm_extended[15:0],16'b0};
		end
		else if(imm_or_data_reg_write & pre_ALUOP[0] & (instruction_reg[3:0] != 4'b1001))begin
			imm_or_read_data <= Read_data1;
		end
		else if(imm_or_data_reg_write & (pre_ALUOP[2] | (pre_ALUOP[0] & (instruction_reg[3:0] == 4'b1001))) )begin
			imm_or_read_data <= Result_ALU;
		end
		else begin
			;
		end
    end
	//这里只是针对一些大的情况把相应的数据写进imm_or_read_data寄存器中，用不用这个寄存器中的数据还不好说
	//最迟在写回阶段前把数据写进这个寄存器中，然后在取指阶段时通过选择，看是否把这个数据写进寄存器中

	assign address_reg = Read_data1;
	assign address_jumplong = {PC[31:28],instruction_reg[25:0],2'b00};
	assign PC_next     =   	 ({32{PC_src[0]}} & Result_ALU)
				|({32{PC_src[1]}} & ALU_out) 
				|({32{PC_src[2]}} & PC)
				|({32{PC_src[3]}} & address_reg)
				|({32{PC_src[4]}} & address_jumplong)
				|({32{PC_src[5]}} & EPC);




	assign Address = {ALU_out[31:2],2'b00};
	Create_Mask create_mask(
		.Result(ALU_out_next),
		.opcode(instruction_reg[31:26]),

		.mask(Mask)
	);

	MemtoReg_Data_Read memtoreg_data_read(
		.mask(Mask),
		.Memdata(mem_data_reg),
		.rt_reg_data(Read_data2),
		.opcode(instruction_reg[31:26]),

		.memtoReg_data(MemtoReg_Data)
	);

	Create_Strb create_strb(
		.Result(ALU_out),
		.opcode(instruction_reg[31:26]),

		.strb(Write_strb)
	);

	RegtoMem_Data_Write regtomem_data_write(
		.strb(Write_strb),
		.rt_reg_data(Read_data2),
	 	.opcode(instruction_reg[31:26]),

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



	assign RF_wdata_temp =  ({32{Reg_src[0]}} & imm_or_read_data) | 
				({32{Reg_src[1]}} & Shift_out) | 
				({32{Reg_src[2]}} & imm_or_read_data) |
				({32{Reg_src[3]}} & imm_or_read_data) |
				({32{Reg_src[4]}} & ALU_out) |
				({32{Reg_src[5]}} & ALU_out_next);

	assign RF_wdata = (MemtoReg) ? MemtoReg_Data : RF_wdata_temp;

	assign RF_waddr = (({5{RegDst[0]}} & instruction_reg[15:11])|
			   ({5{RegDst[1]}} & instruction_reg[20:16])|
			   ({5{RegDst[2]}} & 5'b11111));

	//用Regsrc选择PC_write,Result_shift,Result_ALU,imm后面补充16个0,直接寄存器转移rdata1(rs)
	//用MemtoReg选择MEM中数据和上面选择的结果
	//用RegDst来选择放入rt寄存器，rd寄存器，或31号寄存器

// TODO: Please add your custom CPU code here

endmodule

module control(
	input[31:0]	Result,
	input[31:0]	instruction,
	input[31:0]	rt,
	input [8:0]	current_state,

	input			intr_valid,
	input         	Inst_Valid,

	output			Inst_Req_Valid,
	output			Inst_Ready,
	output			Read_data_Ready,

	output 		 	ALUout_write,
	output 		 	Shiftout_write,
	output 		 	imm_or_data_reg_write,
	output		 	branch_judge,
	output[5:0]	 	PCsrc,
	output  		IRwrite, 
	output[0:0]	 	PCwrite,

    output[7:0]      pre_ALUop,
	output[5:0]      Reg_src,//分别指代（PC+8），SAL,imm，直接的寄存器，ALU_out,ALU_out_next
	output[2:0]      RegDst,//RegDst[0]：写到rd寄存器；RegDst[1]：写到rt寄存器;RegDst[2]:31号寄存器
	output[0:0]      Branch,//无效：无跳转；有效；跳转
	output[0:0]      MemRead,//无效：不读内存；有效：读内存
	output[0:0]      MemtoReg,//无效：ALU算的数->reg；有效：访存->reg
	output[0:0]      MemWrite,//无效：不写内存；有效：写内存
	output[4:0]      ALUsrcB,//无效：立即数不参与运算；有效：立即数参与运算
	output[1:0]      ALUsrcA,//无效：立即数不参与运算；有效：立即数参与运算
	output[0:0]      RegWrite,//无效：不写寄存器；有效：写寄存器
	output[0:0]      shift_src,//无效：立即数不参与运算；有效：立即数参与运算

	output			 intr_open_temp
);
	wire [5:0]		opcode;
	wire [5:0]		func;
	wire [4:0]		instruct_rt;

	wire [0:0]		temp_not_wirte_1;
	wire [0:0]		temp_not_wirte_2;
	wire [0:0]		temp_not_wirte_3;
	wire [0:0]		temp_wirte_J;
	wire [0:0]		temp_wirte_R;
	wire [2:0]      branch_src;

	assign opcode = instruction[31:26];
	assign func = instruction[5:0];
	assign instruct_rt = instruction[20:16];

    assign pre_ALUop[7] = 0;
    assign pre_ALUop[6] = (opcode[5:3] == 3'b101);//I-type内存写指令;
    assign pre_ALUop[5] = (opcode[5:3] == 3'b100);//I-type内存读指令;
    assign pre_ALUop[4] = (opcode[5:3] == 3'b001);//I-type计算指令;
    assign pre_ALUop[3] = (opcode[5:3] == 3'b000) && (opcode[2]);//I-type分支指令;
	assign pre_ALUop[2] = (opcode[5:3] == 3'b000) && (!opcode[2] && opcode[1]);//J-Type指令;
	assign pre_ALUop[1] = (opcode[5:3] == 3'b000) && (opcode[2:0] == 3'b001);//REGIMM指令;
	assign pre_ALUop[0] = (opcode[5:3] == 3'b000) && (opcode[2:0] == 3'b000);//R-Type指令;
	//preALUop负责对所有指令进行划分

	assign temp_not_wirte_1 = (func == 6'b001000) ? 1 : 0;//R-type中jr指令不用写寄存器;
	assign temp_not_wirte_2 = (func == 6'b001010 && rt != 32'b0) ? 1 : 0;//R-type中movz指令rt!=0不用写寄存器;
	assign temp_not_wirte_3 = (func == 6'b001011 && rt == 32'b0) ? 1 : 0;//R-type中movn指令rt==0指令不用写寄存器;
	assign temp_wirte_J = opcode[0] & pre_ALUop[2];//jal指令，为J-type指令考虑opcode最低位为1;
	assign temp_wirte_R = pre_ALUop[0] & (!temp_not_wirte_1) & (!temp_not_wirte_2) & (!temp_not_wirte_3);//R型指令需要写寄存器
	assign RegWrite = (temp_wirte_R | pre_ALUop[4] | pre_ALUop[5] | temp_wirte_J)&(current_state[4]);
	//RegWrite负责控制是否写寄存器

	assign RegDst[1] = pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6];
	assign RegDst[2] = pre_ALUop[2] | (pre_ALUop[0] & func[3:1] == 3'b100);
	assign RegDst[0] = !RegDst[1] & !RegDst[2];
	//RegDst负责控制写rt,rd或31号寄存器

	assign Branch = pre_ALUop[1] | pre_ALUop[3];//这时的Branch 只考虑对beq等分支进行选择，连同后面的judge_branch一起
	assign branch_src[1] = pre_ALUop[2];//使用拼接进行跳转
	assign branch_src[2] = (pre_ALUop[0] & (func[3:1] == 3'b100)) ? 1 : 0;//使用寄存器中的数据进行跳转
	assign branch_src[0] = !branch_src[1] & !branch_src[2];//一般的PC+4与beq等一系列指令的综合(即选择由Branch给出的跳转选择)


	assign MemRead  = pre_ALUop[5]&current_state[5];
	assign MemtoReg = pre_ALUop[5];
	assign MemWrite = pre_ALUop[6]&current_state[3];
	//写内存，读内存，写寄存器的数据是否为都内存的数据

	
	assign	Inst_Req_Valid = current_state[0] & ~intr_valid;
	assign	Inst_Ready = current_state[6] | (!current_state);
	assign	Read_data_Ready = current_state[7] | (!current_state);
	//关于真实内存的信号

	assign ALUsrcB[4] = current_state[1];//译码阶段，将imm_extended写入
	assign ALUsrcB[3] = current_state[6] | 
			   (current_state[2] & 
			   ((pre_ALUop[0]&(func[3:0] == 4'b1001))|(pre_ALUop[2]&(opcode[1:0] == 2'b11))));
			   //等待指令阶段！！！，将PC+4写入，选择为4;或者为执行阶段，需要算jal;
	assign ALUsrcB[2] = pre_ALUop[1] & current_state[2];//执行阶段，regimm与0进行比较
	assign ALUsrcB[1] = (pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6]) & current_state[2];//执行阶段，计算立即数,移位或者不移位
	assign ALUsrcB[0] = (!ALUsrcB[1] & !ALUsrcB[2] & !ALUsrcB[3] & !ALUsrcB[4]) & current_state[2];//执行阶段，计算read_data2
	//imm_extended,4,0,移位(或不移位的)的立即数(负责进行取指令的),寄存器中的数据
	//为什么这里有ALUsrcB[4]与ALUsrcB[1]都可能是imm_extended，但不合并？
	//因为前面是考虑到必须先算分支指令，而后面可能直接与扩展立即数相加（一样是imm_extended），或者是与j_Type的长跳转相加！与前边的分支跳转不一样！

	assign ALUsrcA[0] = ALUsrcB[3] | ALUsrcB[4];
	assign ALUsrcA[1] = !ALUsrcA[0];
	//PC地址，Read_data1

	assign shift_src = pre_ALUop[0] & !func[2];
	//移位器的选择，寄存器中的后五位或是shamt

	assign Reg_src[0] = ((pre_ALUop[0] && (func[3:0] == 4'b1001)) | (pre_ALUop[2] && (opcode[1:0] == 2'b11))) ? 1 : 0;//对应（PC+8）
	assign Reg_src[1] = (pre_ALUop[0] && (func[5:3] == 3'b000)) ? 1 : 0;//选择数据时对应移位后的数据
	assign Reg_src[2] = (pre_ALUop[4] && (opcode[2:0] == 3'b111)) ? 1 : 0;//选择数据时，直接选择立即数后面添0放进寄存器
	assign Reg_src[3] = (pre_ALUop[0] && (!func[5] & (func[3:1] == 3'b101))) ? 1 : 0;//选择数据时，直接将寄存器放进寄存器
	assign Reg_src[4]   =   (pre_ALUop[0] & !Reg_src[0] & !Reg_src[1] & !Reg_src[3]) |
				(pre_ALUop[4] & !Reg_src[2]) |
				(pre_ALUop[2] & (opcode[1:0] == 2'b11)) ? 1 : 0;//选择ALU_out计算出的结果(如果是立即数，也需要即刻写回),为jal或jalr指令
				//凡是需要立马进入写回状态的，都需要在此刻立马写回
	assign Reg_src[5] = !Reg_src[0] & !Reg_src[1] & !Reg_src[2] & !Reg_src[3] & !Reg_src[4];//其他情况均选择ALU_out_next计算出的结果
	//这里只是进行第一次的选择，后面还会用MEM_Read来选择是否是从内存中读的数据
	//关于寄存器数据的选择

	assign PCsrc =	{intr_open_temp,
			branch_src[1] & current_state[2] & !intr_open_temp,
			 branch_src[2] & current_state[2] & !intr_open_temp,
			 (!Branch | !branch_judge) & branch_src[0] & current_state[2] & !intr_open_temp,
			 Branch & branch_judge & current_state[2] & !intr_open_temp,
			 current_state[6] & !intr_open_temp};
	//0位对应是取指阶段，不管则么样，都是PC+4，来自Result_ALU，
	//1位对应PC+imm跳转
 	//2位对应的是直接PC
	//3位对应的是寄存器read_data2
	//4位对应的是拼接


	branch_judge branch_judge0(
		.Result(Result),
		.pre_ALUop(pre_ALUop),
		.opcode(opcode),
		.instruct_rt(instruct_rt),

		.branch_judge(branch_judge)
	);

	assign PCwrite = (Inst_Valid | current_state[2] | intr_open_temp);
	//何时可以重置PC
	assign IRwrite = current_state[6];
	//何时可以写指令寄存器
	assign ALUout_write = current_state[1] | current_state[2];
	assign Shiftout_write = current_state[2];
	assign imm_or_data_reg_write = current_state[2];

	assign intr_open_temp = (opcode == 6'b010000);
endmodule

module ALU_control(
	input[5:0] opcode,
	input[7:0] pre_ALUop,
	input[5:0] func,
	input[8:0] current_state,

	output[2:0] ALUOP
);
	wire[2:0] temp_ALUOP;

	wire[2:0] temp_ALUOP0;//
	wire[2:0] temp_ALUOP1;//先承接I型分支，regimm型;
	wire[2:0] temp_ALUOP2;//再承接I型内存读，内存写;
	wire[2:0] temp_ALUOP3;//再承接I型计算，R-type;
	wire[2:0] judge;//判断怎样构造ALUOP;

	wire[1:0] func_or_opcode_temp;//

	assign temp_ALUOP0 = (pre_ALUop[3]) ? 3'b110 : 3'b000;//考虑是否为I型分支(beq,bne,blez,bgtz,做减法);
	assign temp_ALUOP1 = (pre_ALUop[1]) ? 3'b111 : 3'b000;//考虑是否为regimm型;
	assign temp_ALUOP2 = (pre_ALUop[6] | pre_ALUop[5]) ? 3'b010 : 3'b000;//考虑是否为I型内存读和内存写;

	assign func_or_opcode_temp = ({2{pre_ALUop[0]}} & func[1:0]) | ({2{pre_ALUop[4]}} & opcode[1:0]);
	//方便后面对ALUOP进行构造,选择func[1:0]或opcode[1:0];

	assign judge[0] = (pre_ALUop[4]&(opcode[2:1] == 2'b00)) | (pre_ALUop[0]&(func[3:2] == 2'b00));
	assign judge[1] = (pre_ALUop[4]&(opcode[2] == 1'b1)) | (pre_ALUop[0]&(func[3:2] == 2'b01));
	assign judge[2] = (pre_ALUop[4]&(opcode[2:1] == 2'b01)) | (pre_ALUop[0]&(func[3:2] == 2'b10));
	//judge表示为独热码的形式,判断ALUOP如何构造

	assign temp_ALUOP3  =  ({3{judge[0]}} & {func_or_opcode_temp[1],2'b10}) |
	        	({3{judge[1]}} & {func_or_opcode_temp[1],1'b0,func_or_opcode_temp[0]}) |
			({3{judge[2]}} & {~func_or_opcode_temp[0],2'b11});

	assign temp_ALUOP = temp_ALUOP0 | temp_ALUOP1 | temp_ALUOP2 | temp_ALUOP3;

	assign ALUOP = (
			  current_state[0] | current_state[1] | current_state[6] |
			((current_state[2]) & (pre_ALUop[2] & (opcode[1:0] == 2'b11))) | 
			((current_state[2]) & (pre_ALUop[0] & (func[3:0] == 4'b1001)))
			) ? 3'b010 : temp_ALUOP;
endmodule

module shift_control(
	input [5:0] func,

	output [1:0] shiftop
);
	assign shiftop = func[1:0];//00为左移，11为算术右移，10为逻辑右移

endmodule//给出在移位器中是左移，逻辑或算术右移

module branch_judge(
	input [31:0] Result,
	input [7:0] pre_ALUop,
	input [5:0] opcode,
	input [4:0] instruct_rt,//考虑为rt寄存器对应的位置

	output [0:0] branch_judge
);
	wire [0:0] temp_branch_judge1;
	wire [0:0] temp_branch_judge2;
	wire [0:0] temp_branch_judge3;
	wire [0:0] temp_lez_gtz;//为了后面给出该如何操作，先给出一个桥梁

	assign temp_branch_judge1 = pre_ALUop[1] & (instruct_rt[0]^Result[0]);//R型，bltz,bgez
	assign temp_branch_judge2 = pre_ALUop[3] & !opcode[1] & ((Result == 32'b0)^opcode[0]);//beq,bne
	assign temp_lez_gtz = (!opcode[0] & ((Result == 32'b0) | Result[31])) | (opcode[0] & !(Result == 32'b0) & !Result[31]);
	assign temp_branch_judge3 = pre_ALUop[3] & opcode[1] & temp_lez_gtz;

	assign branch_judge = temp_branch_judge1 | temp_branch_judge2 | temp_branch_judge3;
endmodule
//由ALU算出的result以及overflow等一系列的值，结合beq等指令的操作码，给出在这些情况下是否要实行branch操作
//注意这里仅仅给出branch_judge来判定是否进行分支跳转，计算出的结果不包含跳转地址（？）
//这里其实不用考虑overflow？因为第二个寄存器始终取值为0
//都是有符号数的比较然后跳转？

module sign_extended(
	input [15:0] imm,
	input [5:0] opcode, 
	input [7:0] pre_ALUop,

	output [31:0] imm_extended
);
	wire [0:0] sign;

	assign sign  =  !(pre_ALUop[4] & (
			  (opcode[2:0] == 3'b100) 
			| (opcode[2:0] == 3'b101) 
			| (opcode[2:0] == 3'b110)
			| (opcode[2:0] == 3'b011)));

	assign imm_extended = {{16{sign & imm[15]}},imm};
endmodule

module PC_update(
	input   clk,
	input  [31:0]  PC_next,
	input [0:0] PC_write,
	input [0:0] rst,
	input [8:0] current_state,

	output [31:0]  PC
);

	reg [31:0] PC_reg;

	always @(posedge clk) begin
		if(rst) begin
			PC_reg <= 0;
		end
		else if(current_state[8])begin
			PC_reg <= 32'h100;
		end
		else if(PC_write & !current_state[8])begin
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
	input [5:0] opcode,//考虑lwl与lwr设置掩码的逻辑不一样

	output [3:0] mask
);
	wire [3:0] temp_mask1_lb;
	wire [3:0] temp_mask1_lh;
	wire [3:0] temp_mask1_lw;

	wire [3:0] temp_mask1_lb_lh;

	wire [3:0] temp_mask1;//承接一般的掩码
	wire [3:0] temp_mask2;//承接lwl与lwr对应的掩码

	assign temp_mask1_lb[0] = !Result[0] & !Result[1];
	assign temp_mask1_lb[1] = Result[0] & !Result[1];
	assign temp_mask1_lb[2] = !Result[0] & Result[1];
	assign temp_mask1_lb[3] = Result[0] & Result[1];//lb的掩码

	assign temp_mask1_lh[0] = !Result[1];
	assign temp_mask1_lh[1] = !Result[1];
	assign temp_mask1_lh[2] =  Result[1];
	assign temp_mask1_lh[3] =  Result[1];//lh的掩码

	assign temp_mask1_lw = 4'b1111;

	assign temp_mask1_lb_lh = (opcode[0]) ? temp_mask1_lh : temp_mask1_lb;
	assign temp_mask1 = (opcode[1]) ? temp_mask1_lw : temp_mask1_lb_lh;

	assign temp_mask2[3] = (opcode[2]) ? 1'b1 : (Result[0] & Result[1]);
	assign temp_mask2[2] = (opcode[2]) ? (!Result[1] | !Result[0]) : Result[1];
	assign temp_mask2[1] = (opcode[2]) ? (!Result[1]) : (Result[0] | Result[1]);
	assign temp_mask2[0] = (opcode[2]) ? (!Result[0] & !Result[1]) : 1'b1;

	assign mask = (!opcode[0] & opcode[1]) ? temp_mask2 : temp_mask1;

endmodule

module MemtoReg_Data_Read(
	input [3:0] mask,
	input [31:0] Memdata,
	input [31:0] rt_reg_data,
	input [5:0] opcode,

	output [31:0] memtoReg_data
);
	wire [31:0] temp_memtoReg_data1;
	wire [31:0] temp_memtoReg_data2;
	wire [31:0] temp_memtoReg_data4;

	wire [31:0] temp_memtoReg_data_lb;
	wire [31:0] temp_memtoReg_data_lh;
	wire [31:0] temp_memtoReg_data_lw;
	wire [0:0] sign;//考虑是否需要进行符号扩展

	wire [31:0] memtoReg_lwl;
	wire [31:0] temp_memtoReg_lwl_1;
	wire [31:0] temp_memtoReg_lwl_2;
	wire [31:0] temp_memtoReg_lwl_3;
	wire [31:0] temp_memtoReg_lwl_4;
	wire [31:0] memtoReg_lwr;
	wire [31:0] temp_memtoReg_lwr_1;
	wire [31:0] temp_memtoReg_lwr_2;
	wire [31:0] temp_memtoReg_lwr_3;
	wire [31:0] temp_memtoReg_lwr_4;

	assign sign = (opcode[2] & !opcode[1]) ? 0 : 1;

	assign temp_memtoReg_data1 = (!opcode[0] & !opcode[1]) ? 
				{24'b0,({8{mask[0]}} & Memdata[7:0]) | ({8{mask[1]}} & Memdata[15:8])
				     | ({8{mask[2]}} & Memdata[23:16]) | ({8{mask[3]}} & Memdata[31:24])} : 32'b0;
	//考虑为承接lb与lbu的数值（还未进行扩展）

	assign temp_memtoReg_data_lb = {{24{sign & temp_memtoReg_data1[7]}},temp_memtoReg_data1[7:0]};
	//进行lb的扩展

	assign temp_memtoReg_data2 = (opcode[0] & !opcode[1]) ? 
				{16'b0,({16{mask[0]}} & Memdata[15:0]) | ({16{mask[2]}} & Memdata[31:16])} : 32'b0;
	//考虑为承接lh与lhu的数值（还未进行扩展）

	assign temp_memtoReg_data_lh = {{16{sign & temp_memtoReg_data2[15]}},temp_memtoReg_data2[15:0]};
	//进行lh的扩展

	assign temp_memtoReg_data_lw = (opcode[0] & opcode[1]) ? Memdata : 32'b0;
	//承接lw的取值

	assign temp_memtoReg_data4 = (!opcode[0] & opcode[1]) ? 
				{{8{mask[3]}} & Memdata[31:24] , {8{mask[2]}} & Memdata[23:16]
				     , {8{mask[1]}} & Memdata[15:8] , {8{mask[0]}} & Memdata[7:0]} : 32'b0;
	//考虑为lwl与lwr取出来的值

	assign temp_memtoReg_lwl_1 = (opcode[2:0] == 3'b010)&(!mask[1] & mask[0])
					 ? {Memdata[7:0],rt_reg_data[23:0]} : 32'b0;
	assign temp_memtoReg_lwl_2 = (opcode[2:0] == 3'b010)&(!mask[2] & mask[1])
					 ? {Memdata[15:0],rt_reg_data[15:0]} : 32'b0;
	assign temp_memtoReg_lwl_3 = (opcode[2:0] == 3'b010)&(!mask[3] & mask[2])
					 ? {Memdata[23:0],rt_reg_data[7:0]} : 32'b0;
	assign temp_memtoReg_lwl_4 = (opcode[2:0] == 3'b010)&(mask[3])
					 ? Memdata[31:0] : 32'b0;
	assign memtoReg_lwl = temp_memtoReg_lwl_1 | temp_memtoReg_lwl_2 | temp_memtoReg_lwl_3 | temp_memtoReg_lwl_4;
	//根据掩码设计出可能的lwl后取值

	assign temp_memtoReg_lwr_1 = (opcode[2:0] == 3'b110)&(mask[1] & !mask[0])
					 ? {rt_reg_data[31:24],Memdata[31:8]} : 32'b0;
	assign temp_memtoReg_lwr_2 = (opcode[2:0] == 3'b110)&(mask[2] & !mask[1])
					 ? {rt_reg_data[31:16],Memdata[31:16]} : 32'b0;
	assign temp_memtoReg_lwr_3 = (opcode[2:0] == 3'b110)&(mask[3] & !mask[2])
					 ? {rt_reg_data[31:8],Memdata[31:24]} : 32'b0;
	assign temp_memtoReg_lwr_4 = (opcode[2:0] == 3'b110)&(mask[0])
					 ? Memdata : 32'b0;
	assign memtoReg_lwr = temp_memtoReg_lwr_1 | temp_memtoReg_lwr_2 | temp_memtoReg_lwr_3 | temp_memtoReg_lwr_4;
	//根据掩码设计出可能的lwr后取值

	assign memtoReg_data  = temp_memtoReg_data_lb|
				temp_memtoReg_data_lh|
				temp_memtoReg_data_lw|
				memtoReg_lwl|
				memtoReg_lwr;
endmodule

module Create_Strb(
	input [31:0] Result,
	input [5:0] opcode,//考虑lwl与lwr设置掩码的逻辑不一样

	output [3:0] strb
);
	wire [3:0] temp_strb1_lb;
	wire [3:0] temp_strb1_lh;
	wire [3:0] temp_strb1_lw;

	wire [3:0] temp_strb1_lb_lh;

	wire [3:0] temp_strb1;//承接一般的掩码
	wire [3:0] temp_strb2;//承接lwl与lwr对应的掩码

	assign temp_strb1_lb[0] = !Result[0] & !Result[1];
	assign temp_strb1_lb[1] = Result[0] & !Result[1];
	assign temp_strb1_lb[2] = !Result[0] & Result[1];
	assign temp_strb1_lb[3] = Result[0] & Result[1];//lb的掩码

	assign temp_strb1_lh[0] = !Result[1];
	assign temp_strb1_lh[1] = !Result[1];
	assign temp_strb1_lh[2] =  Result[1];
	assign temp_strb1_lh[3] =  Result[1];//lh的掩码

	assign temp_strb1_lw = 4'b1111;

	assign temp_strb1_lb_lh = (opcode[0]) ? temp_strb1_lh : temp_strb1_lb;
	assign temp_strb1 = (opcode[1]) ? temp_strb1_lw : temp_strb1_lb_lh;

	assign temp_strb2[3] = (opcode[2]) ? 1'b1 : (Result[0] & Result[1]);
	assign temp_strb2[2] = (opcode[2]) ? (!Result[1] | !Result[0]) : Result[1];
	assign temp_strb2[1] = (opcode[2]) ? (!Result[1]) : (Result[0] | Result[1]);
	assign temp_strb2[0] = (opcode[2]) ? (!Result[0] & !Result[1]) : 1'b1;

	assign strb = (!opcode[0] & opcode[1]) ? temp_strb2 : temp_strb1;

endmodule

module RegtoMem_Data_Write(
	input [3:0] strb,
	input [31:0] rt_reg_data,
	input [5:0] opcode,

	output [31:0] Regtomem_data
);

	wire [31:0] temp_Regtomem_data_sb;
	wire [31:0] temp_Regtomem_data_sh;
	wire [31:0] temp_Regtomem_data_sw;

	wire [31:0] Regtomem_swl;
	wire [31:0] Regtomem_swr;
	wire [31:0] temp_Regtomem_swr_1;
	wire [31:0] temp_Regtomem_swr_2;
	wire [31:0] temp_Regtomem_swr_3;
	wire [31:0] temp_Regtomem_swr_4;

	wire [31:0] temp_Regtomem_swl_1;
	wire [31:0] temp_Regtomem_swl_2;
	wire [31:0] temp_Regtomem_swl_3;
	wire [31:0] temp_Regtomem_swl_4;


	assign temp_Regtomem_data_sb = (!opcode[0] & !opcode[1]) ? 
					{{8{strb[3]}} & rt_reg_data[7:0] , {8{strb[2]}} & rt_reg_data[7:0]
				      , {8{strb[1]}} & rt_reg_data[7:0] , {8{strb[0]}} & rt_reg_data[7:0]} : 32'b0;
	//进行lb的扩展

	assign temp_Regtomem_data_sh = (opcode[0] & !opcode[1]) ? 
					{{16{strb[2]}} & rt_reg_data[15:0] , {16{strb[0]}} & rt_reg_data[15:0]} : 32'b0;
	//进行lh的扩展

	assign temp_Regtomem_data_sw = (opcode[0] & opcode[1]) ? rt_reg_data : 32'b0;
	//承接lw的取值

	assign temp_Regtomem_swl_1 = (opcode[2:0] == 3'b010)&(strb[3])
					 ? rt_reg_data : 32'b0;
	assign temp_Regtomem_swl_2 = (opcode[2:0] == 3'b010)&(!strb[3] & strb[2])
					 ? {8'b0,rt_reg_data[31:8]} : 32'b0;
	assign temp_Regtomem_swl_3 = (opcode[2:0] == 3'b010)&(!strb[2] & strb[1])
					 ? {16'b0,rt_reg_data[31:16]} : 32'b0;
	assign temp_Regtomem_swl_4 = (opcode[2:0] == 3'b010)&(!strb[1] & strb[0])
					 ? {24'b0,rt_reg_data[31:24]} : 32'b0;
	assign Regtomem_swl  =   temp_Regtomem_swl_1 |
				 temp_Regtomem_swl_2 | 
				 temp_Regtomem_swl_3 | 
				 temp_Regtomem_swl_4;
	//根据掩码设计出可能的lwl后取值


	assign temp_Regtomem_swr_1 = (opcode[2:0] == 3'b110)&(strb[1] & !strb[0])
					 ? {rt_reg_data[23:0],8'b0} : 32'b0;
	assign temp_Regtomem_swr_2 = (opcode[2:0] == 3'b110)&(strb[2] & !strb[1])
					 ? {rt_reg_data[15:0],16'b0} : 32'b0;
	assign temp_Regtomem_swr_3 = (opcode[2:0] == 3'b110)&(strb[3] & !strb[2])
					 ? {rt_reg_data[8:0],24'b0} : 32'b0;
	assign temp_Regtomem_swr_4 = (opcode[2:0] == 3'b110)&(strb[0])
					 ?  rt_reg_data : 32'b0;
	assign Regtomem_swr  =   temp_Regtomem_swr_1 |
				 temp_Regtomem_swr_2 | 
				 temp_Regtomem_swr_3 | 
				 temp_Regtomem_swr_4;
	//根据掩码设计出可能的lwr后取值

	assign Regtomem_data  = temp_Regtomem_data_sb|
				temp_Regtomem_data_sh|
				temp_Regtomem_data_sw|
				Regtomem_swl|
				Regtomem_swr;
endmodule

module state_change(
	input [31:0] 	instruction,
	input 		clk,
	input		rst,
	input [7:0] 	pre_ALUop,
	input 		intr_valid,

	input 		Inst_Req_Valid,
	input 		Inst_Req_Ready,
	input		Inst_Valid,
	input		Mem_Req_Ready,
	input		Read_data_Valid,

	output [8:0]    current_state
);
	localparam 	Init = 9'b000000000,//新加初始状态
			IFetch = 9'b000000001,//没变,取指
			IWait = 9'b001000000,//第七位，读指令等待
			IDecode = 9'b000000010,//没变，译码
			Execute = 9'b000000100,//没变，执行
			Store = 9'b000001000,//没变，内存写
			Load = 9'b000100000,//第六位，为读内存
			Read_data_wait = 9'b010000000,//第八位，读数据等待
			Write_back = 9'b000010000,//没变，写回
			Interrupt = 9'b100000000;//中断

	reg [8:0] current_state;

	reg [8:0] next_state;

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
				if(intr_valid)begin
					next_state = Interrupt;
				end
				else if(Inst_Req_Valid & Inst_Req_Ready & !intr_valid)begin
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
				if(instruction[31:0] == 32'b0 | instruction[31:26] == 6'b010000)begin
					next_state = IFetch;
				end
				else begin
					next_state = Execute;
				end
			Execute:
				if( pre_ALUop[1] | (pre_ALUop[2] & (instruction[27:26] == 2'b10)) | 
				    pre_ALUop[3]) begin
					next_state = IFetch;
				end
				else if(pre_ALUop[0] | 
					pre_ALUop[4] | (pre_ALUop[2] & (instruction[27:26] == 2'b11))) begin
					next_state = Write_back;
				end
				else if(pre_ALUop[6])begin
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
			Interrupt:
				next_state = IFetch;
			default: 
				next_state = IFetch;
		endcase
	end

endmodule
//状态机只负责进行状态转变以及下一状态的判断

/*`timescale 10ns / 1ns

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
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
  //wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here
/*	wire			RF_wen, IR_wen, PC_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	wire [31:0] rs, rt;
	wire [31:0] new_PC, next_PC, branch_J_PC, branch_R_PC;
	wire [31:0] IR, Mem_out;
	wire [ 5:0] REGsrc, ALUsrc_2;
	wire [11:0] state, next_state;
	wire [ 5:0] sel;
	wire [ 2:0] ALUsrc_1, REGdst, ALUop;
	wire [ 1:0] Shiftop;
	wire [31:0] ALU_result, Shift_result;
	wire [31:0] extend_imm, lui_imm;
	wire [31:0] ALUopr_1, ALUopr_2, Shiftopr_1;
	wire [ 4:0] Shiftopr_2; 
	wire zero, overflow, carryout, logic_extend, addr_wen;
	reg [31:0] EPC;
	reg [31:0] addr_reg, A, B;
	reg [31:0] Mem_result, ALU_out, Shift_out;
	reg [31:0] cycle_cnt, read_mem_cnt, intr_cnt;
	reg reg_zero, mask_intr;
	wire int;

        pc
	    pc_1(
			 clk,
	         rst,
		 	 new_PC,
		 	 PC
		);
	assign new_PC = PC_wen ? next_PC : PC; 

	Instruction_Reg
	    IR_1(
			clk,
			IR_wen,
			Instruction,
			IR
	    );

	reg_file
	    reg_file_1(
			clk, 
	        RF_waddr, 
			IR[25:21], 
			IR[20:16], 
			RF_wen, 
			RF_wdata,
			rs, 
			rt
	    );
	assign RF_waddr = {5{REGdst[2]}} & IR[15:11] | {5{REGdst[1]}} & IR[20:16] | {5{REGdst[0]}} & 5'b11111;
	assign RF_wdata = {32{REGsrc[5]}} & lui_imm | {32{REGsrc[4] | REGsrc[0]}} & ALU_out | {32{REGsrc[3]}} & A | {32{REGsrc[2]}} & Shift_out 
	                | {32{REGsrc[1]}} & Mem_out ;

	always@(posedge clk)
	begin
		if (state[11])
			EPC <= PC;
		else
			EPC <= EPC;
	end

	always@(posedge clk)
	begin
		if (rst)
			mask_intr <= 0;
		else if (state[11])
			mask_intr <= 1;
		else if (state[1] & sel[5])
			mask_intr <= 0;
		else
			mask_intr <= mask_intr;
	end

	always@(posedge clk)
	begin
		A <= rs;
	end

	always@(posedge clk)
	begin
		B <= rt;
	end

	always@(posedge clk)
	begin
		reg_zero <= zero;
	end

	always@(posedge clk)
	begin
		ALU_out <= ALU_result;
	end

	always@(posedge clk)
	begin
		Shift_out <= Shift_result;
	end

	always@(posedge clk)
	begin
		Mem_result <= Read_data;
	end

	always@(posedge clk)
	begin 
		if (addr_wen)
		    addr_reg <= ALU_result;
		else
		    addr_reg <= addr_reg;
	end
	assign Address = {addr_reg[31:2], 2'd0};

	always@(posedge clk)
	begin
		if (rst)
			cycle_cnt <= 32'd0;
		else
			cycle_cnt <= cycle_cnt + 32'd1;
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	always@(posedge clk)
	begin
		if (rst)
			read_mem_cnt <= 32'd0;
		else if (Mem_Req_Ready & MemRead)
			read_mem_cnt <= read_mem_cnt + 32'd1;
		else
			read_mem_cnt <= read_mem_cnt;
	end
	assign cpu_perf_cnt_1 = read_mem_cnt;

	always@(posedge clk)
	begin
		if (rst)
			intr_cnt <= 32'd0;
		else if (state[11])
			intr_cnt <= intr_cnt + 32'd1;
		else
			intr_cnt <= intr_cnt;
	end
	assign cpu_perf_cnt_2 = intr_cnt;


	control_FSM
        fsm_1(
			rst,
			clk,
			next_state,
			state
	    );

	Next_State
	    next_state_1(
			IR,
			IR[31:26],
			IR[5:0],
			state,
			sel,
			next_state,
			Inst_Req_Valid,
			Inst_Req_Ready,
	        Inst_Ready, 
	        Inst_Valid,
	        MemRead,
	        MemWrite,
	        Mem_Req_Ready,
	        Read_data_Valid,
		    Read_data_Ready,
			int
	    );

	control_unit
	    control_unit_1(
			{IR[15:11], IR[5:0]},
			state,
			IR[31:26],
			sel,
			IR[5:0],
			reg_zero,
			intr,
			mask_intr,
			ALUsrc_1,
			ALUsrc_2, 
			IR_wen,
			REGsrc,
			REGdst,
			RF_wen,
			MemRead,
			MemWrite,
			Shiftop,
			Inst_Req_Valid, 
		    Inst_Ready,
		    addr_wen,
	        Read_data_Ready,
			int
	    );

	alu_control
	    alu_control_1(
			state,
			sel,
			IR[31:26],
			IR[5:0],
			ALUop
	    );
	
	branch_control
	    branch_control_1(
			state,
			IR[31:26],
			IR[16],
			IR[5:0],
			zero,
			ALU_result,
			ALU_out,
			branch_J_PC,
			branch_R_PC,
			next_PC,
			PC_wen,
			Inst_Ready,
			Inst_Valid,
			EPC,
			int
	    );
	assign branch_J_PC = {PC[31:28], IR[25:0], 2'b00};
	assign branch_R_PC = A;

	ALU
	    alu(
	        ALUopr_1, 
			ALUopr_2, 
			ALUop, 
			overflow, 
			carryout, 
			zero,
			ALU_result
	    );
	assign logic_extend = IR[29] & IR[28];
	assign lui_imm = {IR[15:0], 16'b0};
    assign extend_imm = logic_extend ? {16'b0, IR[15:0]} : IR[15] ? {{16{1'b1}}, IR[15:0]} : {16'b0, IR[15:0]};
	assign ALUopr_1 = {32{ALUsrc_1[2]}} & PC | {32{ALUsrc_1[1]}} & B | {32{ALUsrc_1[0]}} & A;
	assign ALUopr_2 = {32{ALUsrc_2[5]}} & A | {32{ALUsrc_2[4]}} & 32'd4 | {32{ALUsrc_2[2]}} & B | {32{ALUsrc_2[1]}} & {extend_imm[29:0], 2'b00}
	                | {32{ALUsrc_2[0]}} & extend_imm;
	
	shifter
	    shifter1(
           Shiftopr_1,
	       Shiftopr_2,
	       Shiftop,
	       Shift_result
	    );
	assign Shiftopr_1 = B;
	assign Shiftopr_2 = IR[2] ? A[4:0] : IR[10:6];


	mem_read_modify
	    mem_read_modify_1(
			Mem_result,
			B,
			IR[31:26],
			addr_reg[1:0],
			Mem_out
	    );
	
	mem_write
	    mem_write_1(
			IR[31:26],
			addr_reg[1:0],
			B,
			Write_data,
			Write_strb
	    );

endmodule

module pc(
	input             clk,
	input             rst,
	input [31:0]      new_PC,
	output reg[31:0]   pc
);

    always@(posedge clk)
	begin
		if (rst)
		    pc <= 0;
		else
		    pc <= new_PC;
	end
endmodule

module Instruction_Reg(
	input clk,
	input IR_wen,
	input [31:0] wdata,
	output reg [31:0] Ireg
);

	always@(posedge clk)
	begin
		if (IR_wen)
		    Ireg <= wdata;
	end

endmodule

module control_FSM(
	input rst,
	input clk,
	input [11:0] next_state,
	output reg [11:0] state
);
    always@(posedge clk)
	begin
		if (rst)
	        state <= 12'b000100000000;
		else
		    state <= next_state;
	end

endmodule

module Next_State(
	input  [31:0] IR,
	input  [5:0] opcode,
	input  [5:0] func,
	input  [11:0] state,
	output [5:0] sel,
	output reg [11:0] next_state,
	input Inst_Req_Valid, 
	input Inst_Req_Ready,
	input Inst_Ready, 
	input Inst_Valid,
	input MemRead,
	input MemWrite, 
	input Mem_Req_Ready, 
	input Read_data_Valid,
	input Read_data_Ready,
	input int
);
        localparam Ifech     = 12'b000000000001,
                   Ideco     = 12'b000000000010,
                   Branch_I  = 12'b000000000100,
                   Branch_J  = 12'b000000001000,
                   Execu     = 12'b000000010000,
                   MEmread   = 12'b000000100000,
                   MEmwrite  = 12'b000001000000,
                   Rwrite    = 12'b000010000000,
				   Init      = 12'b000100000000,
				   Iwait     = 12'b001000000000,
				   Rdwait    = 12'b010000000000,
				   INTR      = 12'b100000000000;
                
	assign sel[0] = ~|opcode;  //R型指令
	assign sel[1] = (~|opcode[5:4]) & opcode[3];  //I型运算指令
	assign sel[2] = opcode[5] & ~opcode[3];  //取数指令
	assign sel[3] = opcode[5] & opcode[3];  //存数指令
	assign sel[4] = (~|opcode[5:3]) & (|opcode[2:0]);  //跳转指令
	assign sel[5] = ~opcode[5] & opcode[4];  //ERET指令

    always@(*)
	begin
		case (state)
			Ifech:begin
				  if (int)
				  		next_state = INTR;
			      else if (Inst_Req_Valid & Inst_Req_Ready)
				        next_state = Iwait;
				  else
				        next_state = Ifech;
			end
			Iwait:begin 
				  if (Inst_Ready & Inst_Valid)
				      next_state = Ideco;
				  else
				      next_state = Iwait;
			end
			Branch_I:
			      next_state = Ifech;
			Branch_J:begin
				if (opcode[0] | sel[0])
			            next_state = Rwrite;
				else
				    next_state = Ifech;
			end
			Execu:begin
			      if (sel[3])
			          next_state = MEmwrite;
			      else if (sel[2])
			          next_state = MEmread;
			      else if (sel[1] | sel[0])
			          next_state = Rwrite;
			      else
			          next_state = 32'dx;
			end
			MEmread:begin
			      if (MemRead & Mem_Req_Ready & ~int)
				      next_state = Rdwait;
				  else
				      next_state = MEmread;
			end
			Rdwait:begin
				  if (Read_data_Valid & Read_data_Ready)
				      next_state = Rwrite;
				  else
				      next_state = Rdwait;
			end
			MEmwrite:
			      if (MemWrite & Mem_Req_Ready)
			          next_state = Ifech;
				  else
				      next_state = MEmwrite;
			Rwrite:
			      next_state = Ifech;
			Ideco:begin
				if (sel[4] & ~opcode[2] & opcode[1] | sel[0] & func[3] & ~func[1])  //J型指令或jr、jalr
				    next_state = Branch_J;
				else if (sel[4] & (opcode[2] | ~opcode[1]))  //I型分支指令、REGIMM型指令
				    next_state = Branch_I;
				else if ((|sel[3:1] | sel[0] & (~func[3] | func[1])) & |IR)  //访存、R型运算、移位、条件传输指令
				    next_state = Execu;
				else
				    next_state = Ifech;  //nop指令或ERET指令
			end
			INTR:begin
				next_state = Ifech;
			end
			default:
			      next_state = Ifech;
		endcase
	end
endmodule

module control_unit(
	input [10:0] ins,
	input [11:0] state,
	input [5:0] opcode,
	input [5:0] sel,
	input [5:0] func,
	input zero_reg,
	input intr,
	input mask_intr,
	output [2:0] ALUsrc_1,
	output [5:0] ALUsrc_2,
	output IR_wen,
	output [5:0] REGsrc,
	output [2:0] REGdst,
	output RF_wen,
	output MemRead,
	output MemWrite,
	output [1:0] Shiftop, 
	output Inst_Req_Valid,
	output Inst_Ready,
	output addr_wen, 
	output Read_data_Ready,
	output int
);
	wire movz, movn, jr;
	wire [2:0] R_wen;

	assign ALUsrc_1[2] = state[1] | state[9] | state[3];  //指令等待、译码阶段、J型分支阶段，ALU第一操作数为PC
	assign ALUsrc_1[1] = state[4] & sel[0] & ~func[5] & func[3] & func[1] | state[2] & sel[4] & &opcode[2:1];  //条件传输、blez、bgtz指令，ALU第一操作数为rt
	assign ALUsrc_1[0] = ~(ALUsrc_1[2] | ALUsrc_1[1]);  //其他情况,ALU第一操作数为rs

    assign ALUsrc_2[5] =  state[2] & sel[4] & &opcode[2:1];  //blez、bgtz指令，ALU第二操作数为rs
	assign ALUsrc_2[4] = state[9] | state[3];  //指令等待、J型分支阶段，ALU第二操作数为4
	assign ALUsrc_2[3] = state[2] & sel[4] & ~opcode[2] & ~opcode[1] & opcode[0];  //Branch_I阶段，REGIMM型指令，ALU第二操作数为0
	assign ALUsrc_2[2] = state[4] & sel[0] | state[2] & sel[4] & opcode[2];  //执行阶段，R型、I型分支指令，ALU第二操作数为rt
	assign ALUsrc_2[1] = state[1];  //译码阶段，ALU第二操作数为imm << 2
	assign ALUsrc_2[0] = ~|ALUsrc_2[5:1];  //其他情况，ALU第二操作数为extend-imm
	
	assign IR_wen = state[9];  //指令等待阶段，IR寄存器写使能

    assign movz = sel[0] & ~func[5] & func[3] & func[1] & ~func[0];
	assign movn = sel[0] & ~func[5] & func[3] & func[1] & func[0];
	assign jr = sel[0] & func[3] & ~func[1] & ~func[0];

    assign R_wen[2] = ~movz & ~movn;
	assign R_wen[1] = ~jr;  //jal、jalr指令写寄存器
	assign R_wen[0] = movz & zero_reg | movn & ~zero_reg;  //条件传输写寄存器
	assign RF_wen = state[7] & (R_wen[2] & R_wen[1] | R_wen[0]);

	assign REGsrc[5] = &opcode[3:0] & sel[1];  //lui指令，写REG结果来自立即数
	assign REGsrc[4] = ~|opcode[5:2] & opcode[1] | sel[0] & func[3] & ~func[1];  //jal和jalr指令，写REG结果来自PC+8
	assign REGsrc[3] = sel[0] & ~func[5] & func[3] & func[1];  //mov类指令，写REG结果可能来自rs
	assign REGsrc[2] = ~|func[5:3] & sel[0];  //移位指令，写REG结果来自Shiftresult
	assign REGsrc[1] = sel[2];  //取数指令，写REG结果来自内存
	assign REGsrc[0] = ~|REGsrc[5:1];  //其他指令，写REG结果来自ALU

	assign REGdst[2] = sel[0];  //R型指令，写rd
	assign REGdst[1] = sel[2] | sel[1];  //I型运算指令或取数指令，写rt
	assign REGdst[0] = ~|REGdst[2:1];  //jal指令，写GPR31

    assign Shiftop = {func[1], func[0]};
	assign MemRead = state[5];
	assign MemWrite = state[6];

	assign Inst_Req_Valid = state[0] & ~int;
	assign Inst_Ready = state[9] | state[8];
	assign addr_wen = state[4] & |sel[3:2];
	assign Read_data_Ready = state[10] | state[8];
	assign int = intr & ~mask_intr;

endmodule

module branch_control(
	input [11:0] state,
	input [5:0] opcode,
	input REG,
	input [5:0] func,
	input zero,
	input [31:0] ALU_result,
	input [31:0] ALU_out,
	input [31:0] branch_J_PC,
	input [31:0] branch_R_PC,
	output [31:0] next_PC,
	output PC_wen,
	input Inst_Ready,
	input Inst_Valid,
	input [31:0] EPC,
	input int
);
    wire PC_write_1, PC_write_2, PC_write_3, PC_write_4;

    assign PC_write_1 = state[3] | state[9] & Inst_Ready & Inst_Valid;  //指令等待且握手握中、J型跳转阶段写PC

	assign bltz = ALU_result[0] & ~|opcode[2:1] & opcode[0] & ~REG;
	assign bgez = ~ALU_result[0] & ~|opcode[2:1] & opcode[0] & REG;
	assign beq  = zero & opcode[2] & ~|opcode[1:0];
	assign bne  = ~zero & opcode[2] & ~opcode[1] & opcode[0];
	assign blez = ~ALU_result[0] & &opcode[2:1] & ~opcode[0];
	assign bgtz = ALU_result[0] & &opcode[2:0];
	assign PC_write_2 = state[2] & (bltz | bgez | beq | bne | blez | bgtz); 
	assign PC_write_3 = state[11];  //中断阶段，将0x100写入PC
	assign PC_write_4 = state[1] & ~opcode[5] & opcode[4];  //译码阶段，ERET指令，PC写入程序断点

	assign PC_wen = (PC_write_1 | PC_write_2) | (PC_write_3 | PC_write_4);
	assign next_PC = {32{state[9]}} & ALU_result | {32{state[2]}} & ALU_out 
				   | {32{state[3]}} & ({32{opcode[1]}} & branch_J_PC | ~{32{opcode[1]}} & branch_R_PC) 
	               | {32{state[11]}} & 32'h100 | {32{state[1]}} & EPC;
endmodule

module alu_control(
	input [11:0] state,
	input [5:0] sel,
	input [5:0] opcode,
	input [5:0] func,
	output [2:0] ALUop
);
        localparam add_ALUop = 3'b010,
                   sub_ALUop = 3'b110,
                   slt_ALUop = 3'b111;

    wire [2:0] ALUop_1, ALUop_2;
	wire [2:0] sel_R, sel_I, sel_B;
	wire [2:0] ALUop_R, ALUop_I, ALUop_B;

	assign ALUop_1 = {3{state[9] | state[1] | state[3]}} & add_ALUop;

	assign sel_R[2] = ~|func[3:2];
	assign sel_R[1] = ~func[3] & func[2];
	assign sel_R[0] = func[3] & ~func[2] & func[5];
	assign ALUop_R = {3{sel_R[2]}} & {func[1], 2'b10} | {3{sel_R[1]}} & {func[1], 1'b0, func[0]} | {3{sel_R[0]}} & {~func[0], 2'b11};

	assign sel_I[2] = ~|opcode[2:1];
	assign sel_I[1] = opcode[2];
	assign sel_I[0] = ~opcode[2] & opcode[1];
	assign ALUop_I = {3{sel_I[2]}} & {opcode[1], 2'b10} | {3{sel_I[1]}} & {opcode[1], 1'b0, opcode[0]} | {3{sel_I[0]}} & {~opcode[0], 2'b11};

	assign sel_B[1] = opcode[2] & ~opcode[1];
	assign sel_B[0] = ~opcode[2] | opcode[1];
	assign ALUop_B = {3{sel_B[1]}} & sub_ALUop | {3{sel_B[0]}} & slt_ALUop;
	assign ALUop_2 = {3{state[4]}} & ({3{sel[0]}} & ALUop_R | {3{sel[1]}} & ALUop_I | {3{|sel[3:2]}} & add_ALUop) | {3{state[2]}} & {3{sel[4]}} & ALUop_B;
	assign ALUop = ALUop_1 | ALUop_2;
endmodule

module mem_read_modify(
	input [31:0] Rdata,
	input [31:0] rt,
	input [5:0] opcode,
	input [1:0] ALUresult,
	output [31:0] mem_rs
);
    wire [31:0] lb_0, lb_1, lb_2, lb_3;
	wire [31:0] lh_0, lh_1;
	wire [31:0] lwl_0, lwl_1, lwl_2, lwl_3;
	wire [31:0] lwr_0, lwr_1, lwr_2, lwr_3;
	wire [31:0] lb, lh, lw, lwl, lwr;
	wire [3:0] sel;
	wire [4:0] sel_1;

    assign sel[3] = ALUresult[1] & ALUresult[0];  //地址最后两位2b'11
	assign sel[2] = ALUresult[1] & ~ALUresult[0];  //地址最后两位2b'10
	assign sel[1] = ~ALUresult[1] & ALUresult[0];  //地址最后两位2b'01
	assign sel[0] = ~ALUresult[1] & ~ALUresult[0];  //地址最后两位2b'00

    assign lb_0 = opcode[2] ? {24'b0, Rdata[7:0]} : Rdata[7] ? {{24{1'b1}}, Rdata[7:0]} : {24'b0, Rdata[7:0]};
	assign lb_1 = opcode[2] ? {24'b0, Rdata[15:8]} : Rdata[15] ? {{24{1'b1}}, Rdata[15:8]} : {24'b0, Rdata[15:8]};
	assign lb_2 = opcode[2] ? {24'b0, Rdata[23:16]} : Rdata[23] ? {{24{1'b1}}, Rdata[23:16]} : {24'b0, Rdata[23:16]};
	assign lb_3 = opcode[2] ? {24'b0, Rdata[31:24]} : Rdata[31] ? {{24{1'b1}}, Rdata[31:24]} : {24'b0, Rdata[31:24]};
	assign lb = {32{sel[3]}} & lb_3 | {32{sel[2]}} & lb_2 | {32{sel[1]}} & lb_1 | {32{sel[0]}} & lb_0;

	assign lh_0 = opcode[2] ? {16'b0, Rdata[15:0]} : Rdata[15] ? {{16{1'b1}}, Rdata[15:0]} : {16'b0, Rdata[15:0]};
	assign lh_1 = opcode[2] ? {16'b0, Rdata[31:16]} : Rdata[31] ? {{16{1'b1}}, Rdata[31:16]} : {16'b0, Rdata[31:16]};
	assign lh = {32{sel[2]}} & lh_1 | {32{sel[0]}} & lh_0;

	assign lw = Rdata;

	assign lwl_0 = {Rdata[7:0], rt[23:0]};
	assign lwl_1 = {Rdata[15:8], Rdata[7:0], rt[15:0]}; 
	assign lwl_2 = {Rdata[23:16], Rdata[15:8], Rdata[7:0], rt[7:0]};
	assign lwl_3 = Rdata;
	assign lwl = {32{sel[3]}} & lwl_3 | {32{sel[2]}} & lwl_2 | {32{sel[1]}} & lwl_1 | {32{sel[0]}} & lwl_0;

	assign lwr_0 = Rdata;
	assign lwr_1 = {rt[31:24],Rdata[31:24],Rdata[23:16], Rdata[15:8]};
	assign lwr_2 = {rt[31:16], Rdata[31:24], Rdata[23:16]};
	assign lwr_3 = {rt[31:8], Rdata[31:24]};
	assign lwr = {32{sel[3]}} & lwr_3 | {32{sel[2]}} & lwr_2 | {32{sel[1]}} & lwr_1 | {32{sel[0]}} & lwr_0;

	assign sel_1[4] = ~opcode[1] & ~opcode[0];  //lb
	assign sel_1[3] = ~opcode[1] & opcode[0];  //lh
	assign sel_1[2] = opcode[1] & opcode[0];  //lw
	assign sel_1[1] = ~opcode[2] & opcode[1] & ~opcode[0];  //lwl
	assign sel_1[0] = opcode[2] & opcode[1] & ~opcode[0];  //lwr

	assign mem_rs = {32{sel_1[4]}} & lb | {32{sel_1[3]}} & lh | {32{sel_1[2]}} & lw | {32{sel_1[1]}} & lwl | {32{sel_1[0]}} & lwr;
endmodule

module mem_write(
       input [5:0] opcode,
       input [1:0] ALUresult,
       input [31:0] rt,
       output [31:0] Write_data,
       output [3:0] Write_strb
);
       wire [4:0] sel_1;
       wire [3:0] sel;
       wire [31:0] sb, sb_0, sb_1, sb_2, sb_3;
       wire [31:0] sh, sh_0, sh_1, sw;
       wire [3:0] sb_strb_0, sb_strb_1, sb_strb_2, sb_strb_3, sb_strb;
       wire [3:0] sh_strb_0, sh_strb_1, sh_strb, sw_strb;
       wire [31:0] swl, swl_0, swl_1, swl_2, swl_3;
       wire [31:0] swr, swr_0, swr_1, swr_2, swr_3; 
       wire [3:0] swl_strb_0, swl_strb_1, swl_strb_2, swl_strb_3, swl_strb;
       wire [3:0] swr_strb_0, swr_strb_1, swr_strb_2, swr_strb_3, swr_strb;

       assign sel[3] = ALUresult[1] & ALUresult[0]; 
       assign sel[2] = ALUresult[1] & ~ALUresult[0];
       assign sel[1] = ~ALUresult[1] & ALUresult[0];  
       assign sel[0] = ~ALUresult[1] & ~ALUresult[0]; 

       assign sb_0 = rt;
       assign sb_1[15:8] = rt[7:0];
       assign sb_2[23:16] = rt[7:0];
       assign sb_3[31:24] = rt[7:0];
       assign sb = {32{sel[3]}} & sb_3 | {32{sel[2]}} & sb_2 | {32{sel[1]}} & sb_1 | {32{sel[0]}} & sb_0;

       assign sb_strb_0 = 4'b0001;
       assign sb_strb_1 = 4'b0010;
       assign sb_strb_2 = 4'b0100;
       assign sb_strb_3 = 4'b1000;
       assign sb_strb = {32{sel[3]}} & sb_strb_3 | {32{sel[2]}} & sb_strb_2 | {32{sel[1]}} & sb_strb_1 | {32{sel[0]}} & sb_strb_0;

       assign sh_0 = rt;
       assign sh_1[31:16] = rt[15:0];
       assign sh = {32{sel[2]}} & sh_1 | {32{sel[0]}} & sh_0;

       assign sh_strb_0 = 4'b0011;
       assign sh_strb_1 = 4'b1100;
       assign sh_strb = {32{sel[2]}} & sh_strb_1 | {32{sel[0]}} & sh_strb_0;

       assign sw = rt;
       assign sw_strb = 4'b1111;

       assign swl_0[7:0] = rt[31:24];
       assign swl_1[15:0] = rt[31:16];
       assign swl_2[23:0] = rt[31:8];
       assign swl_3 = rt;
       assign swl = {32{sel[3]}} & swl_3 | {32{sel[2]}} & swl_2 | {32{sel[1]}} & swl_1 | {32{sel[0]}} & swl_0;

       assign swl_strb_0 = 4'b0001;
       assign swl_strb_1 = 4'b0011;
       assign swl_strb_2 = 4'b0111;
       assign swl_strb_3 = 4'b1111;
       assign swl_strb = {32{sel[3]}} & swl_strb_3 | {32{sel[2]}} & swl_strb_2 | {32{sel[1]}} & swl_strb_1 | {32{sel[0]}} & swl_strb_0;

       assign swr_0 = rt;
       assign swr_1[31:8] = rt[23:0];
       assign swr_2[31:16] = rt[15:0];
       assign swr_3[31:24] = rt[7:0];
       assign swr = {32{sel[3]}} & swr_3 | {32{sel[2]}} & swr_2 | {32{sel[1]}} & swr_1 | {32{sel[0]}} & swr_0;

       assign swr_strb_0 = 4'b1111;
       assign swr_strb_1 = 4'b1110;
       assign swr_strb_2 = 4'b1100;
       assign swr_strb_3 = 4'b1000;
       assign swr_strb = {32{sel[3]}} & swr_strb_3 | {32{sel[2]}} & swr_strb_2 | {32{sel[1]}} & swr_strb_1 | {32{sel[0]}} & swr_strb_0;

       assign sel_1[4] = ~opcode[1] & ~opcode[0];  //sb
       assign sel_1[3] = ~opcode[1] & opcode[0];  //sh
       assign sel_1[2] = opcode[1] & opcode[0];  //sw
       assign sel_1[1] = ~opcode[2] & opcode[1] & ~opcode[0];  //swl
       assign sel_1[0] = opcode[2] & opcode[1] & ~opcode[0];  //swr

       assign Write_data = {32{sel_1[4]}} & sb | {32{sel_1[3]}} & sh | {32{sel_1[2]}} & sw | {32{sel_1[1]}} & swl | {32{sel_1[0]}} & swr;
       assign Write_strb = {32{sel_1[4]}} & sb_strb | {32{sel_1[3]}} & sh_strb | {32{sel_1[2]}} & sw_strb | {32{sel_1[1]}} & swl_strb | {32{sel_1[0]}} & swr_strb;
endmodule*/





