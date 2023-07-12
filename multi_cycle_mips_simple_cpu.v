`timescale 10ns / 1ns

module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output 	 	  MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire		RF_wen;
	wire [4:0]	RF_waddr;
	wire [31:0]	RF_wdata;

	wire [31:0]	RF_wdata_temp;//考虑先由

	wire [31:0]	PC_next;

	wire [7:0]	pre_ALUOP;
	wire[5:0]      Reg_src;
	wire[2:0]      RegDst;
	wire[0:0]      Branch;
	wire[0:0]      MemtoReg;
	wire[0:0]      shift_src;
	wire[2:0]      branch_src;

	wire [31:0] Read_data1;
	wire [31:0] Read_data2;
	wire [31:0] imm_extended;//得到进行扩展之后的32位立即数
	wire [31:0] imm_extended_jmp;//得到进行扩展之后的32位立即数,左移两位，准备进行跳转指令
	wire [31:0] address_temp;//PC+4与32位立即数的相加进行选择
	wire [31:0] address_reg;//承接寄存器的跳转
	wire [31:0] address_jumplong;//承接26位长跳转
	//用alu_src与shift_src在上面进行选择

	wire [2:0] ALUOP;//输出ALUOP
	wire [1:0] shiftOP;//输出shiftOP

	wire [31:0] ALU_data1;
	wire [31:0] ALU_data2;//用在ALU中进行计算
	wire [31:0] shift_data1;
	wire [31:0] shift_data2;//用在shifter中进行计算

	wire [31:0] Result_shift;//用在shifter中进行计算
	wire [31:0] Result_ALU;//用在ALU中进行计算
	wire [0:0] Overflow;//用在ALU中进行计算
	wire [0:0] CarryOut;//用在ALU中进行计算
	wire [0:0] Zero;//用在ALU中进行计算

	wire [0:0] branch_judge;//用在判断beq等指令是否需要跳转
	wire [3:0] Mask;//用在ALU中进行计算得到从内存中取的地址
	//以上两者均需要ALU的结果

	wire [31:0] MemtoReg_Data;//通过address中取出的数，制作出应该放进rt寄存器种的数


	wire [4:0]current_state;
	wire [31:0] imm_adder;
	reg [31:0] instruction_reg;//多周期存放
	reg [31:0] read_dataA_reg;//多周期存放
	reg [31:0] read_dataB_reg;//多周期存放
	reg [31:0] ALU_out;
	reg [31:0] ALU_out_next;
	reg [31:0] Shift_out;
	reg [31:0] mem_data_reg;
	reg [31:0] imm_or_read_data;

	wire[4:0]      ALUsrcB;
	wire[1:0]      ALUsrcA;
	wire[0:0] PC_write;
	wire[0:0] IR_wrcd ~/cod-lab && git add software/workload/ucas-cod/benchmark/simple_test/common/perf_cnt.cite;

	wire [4:0]PC_src;

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

		.current_state(current_state),
		.IRwrite(IR_write)
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
        end;

	control control0(
		.opcode(instruction_reg[31:26]),
		.func(instruction_reg[5:0]),
		.rt(Read_data2),
		.current_state(current_state),

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
		.branch_src(branch_src)
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

	assign imm_extended_jmp = {imm_extended[29:0],2'b0};//要跳转的数，对应的是取数存数指令
	assign imm_adder = (pre_ALUOP[3]) ? imm_extended_jmp : imm_extended;
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
		ALU_out <= Result_ALU;
        end

	always @(posedge clk) begin
		Shift_out <= Result_shift;
        end

	always @(posedge clk) begin
		if(pre_ALUOP[4])begin
			imm_or_read_data <= {imm_extended[15:0],16'b0};
		end
		else if(pre_ALUOP[0] & (instruction_reg[3:0] != 4'b1001))begin
			imm_or_read_data <= Read_data1;
		end
		else if(pre_ALUOP[2] | (pre_ALUOP[0] & (instruction_reg[3:0] == 4'b1001)))begin
			imm_or_read_data <= Result_ALU;
		end
		else begin
			;
		end
        end
	//这里只是针对一些大的情况把相应的数据写进imm_or_read_data寄存器中，用不用这个寄存器中的数据还不好说
	//最迟在写回阶段前把数据写进这个寄存器中，然后在取指阶段时通过选择，看是否把这个数据写进寄存器中

	branch_judge branch_judge0(
		.Result(Result_ALU),
		.pre_ALUop(pre_ALUOP),
		.opcode(instruction_reg[31:26]),
		.instruct_rt(instruction_reg[20:16]),

		.branch_judge(branch_judge)
	);

	assign PC_src ={branch_src[1] & current_state[2],
			branch_src[2] & current_state[2],
			(!Branch | !branch_judge) & !branch_src[1] & !branch_src[2] & current_state[2],
			Branch & branch_judge & current_state[2],
			current_state[0]};
	//0位对应是取指阶段，不管则么样，都是PC+4，来自Result_ALU，
	//1位对应PC+imm跳转
	//2位对应的是直接PC
	//3位对应的是寄存器read_data2
	//4位对应的是拼接
	assign address_reg = Read_data1;
	assign address_jumplong = {PC[31:28],instruction_reg[25:0],2'b00};
	assign PC_next     =   	 ({32{PC_src[0]}} & Result_ALU)
				|({32{PC_src[1]}} & ALU_out) 
				|({32{PC_src[2]}} & PC)
				|({32{PC_src[3]}} & address_reg)
				|({32{PC_src[4]}} & address_jumplong);

	/*assign imm_extended_jmp = imm_extended << 2;
	assign address_temp = (Branch & branch_judge) ? (PC_normal_next + imm_extended_jmp) : PC_normal_next;
	assign address_reg = Read_data1;
	assign address_jumplong = {PC_normal_next[31:28],Instruction[25:0],2'b00} ;
	assign PC_next = ({32{branch_src[0]}} & address_temp) 
			|({32{branch_src[1]}} & address_jumplong) 
			|({32{branch_src[2]}} & address_reg) ;*/
	//以上位通过Result完成branch的部分

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

	//用Regsrc选择PC_write,Result_shift,Result_ALU,imm后面补充16个0,直接寄存器转移rdata1(rs)
	//用MemtoReg选择MEM中数据和上面选择的结果
	//用RegDst来选择放入rt寄存器，rd寄存器，或31号寄存器

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
			  
	// TODO: PLEASE ADD YOUR CODE BELOW
	
endmodule

module control(
	input[5:0]      opcode,
	input[5:0]	func,
	input[31:0]	rt,
	input [4:0]	current_state,

	output[0:0]	 PCwrite,
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
	output[2:0]      branch_src//无效：立即数不参与运算；有效：立即数参与运算
);
	wire [0:0]temp_not_wirte_1;
	wire [0:0]temp_not_wirte_2;
	wire [0:0]temp_not_wirte_3;
	wire [0:0]temp_wirte_J;
	wire [0:0]temp_wirte_R;

        assign pre_ALUop[7] = 0;
        assign pre_ALUop[6] = (opcode[5:3] == 3'b101);//I-type内存写指令;
        assign pre_ALUop[5] = (opcode[5:3] == 3'b100);//I-type内存读指令;
        assign pre_ALUop[4] = (opcode[5:3] == 3'b001);//I-type计算指令;
        assign pre_ALUop[3] = (opcode[5:3] == 3'b000) && (opcode[2]);//I-type分支指令;
	assign pre_ALUop[2] = (opcode[5:3] == 3'b000) && (!opcode[2] && opcode[1]);//J-Type指令;
	assign pre_ALUop[1] = (opcode[5:3] == 3'b000) && (opcode[2:0] == 3'b001);//REGIMM指令;
	assign pre_ALUop[0] = (opcode[5:3] == 3'b000) && (opcode[2:0] == 3'b000);//R-Type指令;

	assign temp_not_wirte_1 = (func == 6'b001000) ? 1 : 0;//R-type中jr指令不用写寄存器;
	assign temp_not_wirte_2 = (func == 6'b001010 && rt != 32'b0) ? 1 : 0;//R-type中movz指令rt!=0不用写寄存器;
	assign temp_not_wirte_3 = (func == 6'b001011 && rt == 32'b0) ? 1 : 0;//R-type中movn指令rt==0指令不用写寄存器;
	assign temp_wirte_J = opcode[0] & pre_ALUop[2];//jal指令，为J-type指令考虑opcode最低位为1;
	assign temp_wirte_R = pre_ALUop[0] & (!temp_not_wirte_1) & (!temp_not_wirte_2) & (!temp_not_wirte_3);//R型指令需要写寄存器

	assign RegDst[1] = pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6];
	assign RegDst[2] = pre_ALUop[2] | (pre_ALUop[0] & func[3:1] == 3'b100);
	assign RegDst[0] = !RegDst[1] & !RegDst[2];
	assign Branch = pre_ALUop[1] | pre_ALUop[3];
	//这时的branch 只考虑对beq等分支进行选择，连同后面的judge_branch一起
	assign MemRead  = pre_ALUop[5]&current_state[3];
	assign MemtoReg = pre_ALUop[5];
	assign MemWrite = pre_ALUop[6]&current_state[3];

	assign ALUsrcB[4] = current_state[1];//译码阶段，将imm_extended写入
	assign ALUsrcB[3] = current_state[0] | 
			   (current_state[2] & 
			   ((pre_ALUop[0]&(func[3:0] == 4'b1001))|(pre_ALUop[2]&(opcode[1:0] == 2'b11))));
			   //取指阶段，将PC+4写入，选择为4;或者为执行阶段，需要算jal;
	assign ALUsrcB[2] = pre_ALUop[1] & current_state[2];//执行阶段，regimm与0进行比较
	assign ALUsrcB[1] = (pre_ALUop[4] | pre_ALUop[5] | pre_ALUop[6]) & current_state[2];//执行阶段，计算立即数,移位或者不移位
	assign ALUsrcB[0] = (!ALUsrcB[1] & !ALUsrcB[2] & !ALUsrcB[3] & !ALUsrcB[4]) & current_state[2];//执行阶段，计算read_data2

	assign ALUsrcA[0] = ALUsrcB[3] | ALUsrcB[4];
	assign ALUsrcA[1] = !ALUsrcA[0];

	assign RegWrite = (temp_wirte_R | pre_ALUop[4] | pre_ALUop[5] | temp_wirte_J)&(current_state[4]);
	assign shift_src = pre_ALUop[0] & !func[2];
	assign PCwrite = (current_state[0] | (current_state[2]));

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

	assign branch_src[1] = pre_ALUop[2];//使用拼接进行跳转
	assign branch_src[2] = (pre_ALUop[0] & (func[3:1] == 3'b100)) ? 1 : 0;//使用寄存器中的数据进行跳转
	assign branch_src[0] = !branch_src[1] & !branch_src[2];//一般的PC+4与beq等一系列指令的综合

endmodule

module ALU_control(
	input[5:0] opcode,
	input[7:0] pre_ALUop,
	input[5:0] func,
	input[4:0] current_state,

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
			  current_state[0] | current_state[1] | 
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

	output [31:0]  PC
);

	reg [31:0] PC_reg;

	always @(posedge clk) begin
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

	output [4:0]    current_state,
	output 		IRwrite
);
	localparam 	Init = 8'b00000000,//新加初始状态
			IFetch = 8'b00000001,//没变
			IWait = 8'b01000000,//第七位，读指令等待
			IDecode = 8'b00000010,//没变
			Execute = 8'b00000100,//没变
			Store = 8'b00001000,//没变
			Load = 8'b00100000,//第六位，为读内存
			Read_data_wait = 8'b10000000,//第八位，读数据等待
			Write_back = 8'b00010000, //没变

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
				next_state = 5'b00010;
			5'b00010:
				if(instruction == 32'b0) begin
					next_state = 5'b00001;
				end
				else begin
					next_state = 5'b00100;
				end
			5'b00100:
				if( pre_ALUop[1] | (pre_ALUop[2] & (instruction[27:26] == 2'b10)) | 
				    pre_ALUop[3]) begin
					next_state = 5'b00001;
				end
				else if(pre_ALUop[0] | 
					pre_ALUop[4] | (pre_ALUop[2] & (instruction[27:26] == 2'b11))) begin
					next_state = 5'b10000;
				end
				else begin
					next_state = 5'b01000;
				end
			5'b01000:
				if(pre_ALUop[5]) begin
					next_state = 5'b10000;
				end
				else if(pre_ALUop[6]) begin
					next_state = 5'b00001;
				end
				else begin
					next_state = 5'b01000;
				end
			5'b10000: 
				next_state = 5'b00001;
			default: 
				next_state = 5'b00001;
		endcase
	end


	assign	IRwrite = current_state[0];


endmodule
