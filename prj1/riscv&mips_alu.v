/*`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module ALU(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	wire CO;
	wire cin;

	wire [3:0] judge_ALU;
	wire [32:0]temp_and;
	wire [32:0]temp_or;
	wire [32:0]temp_xor;

	wire [31:0]temp_sub0;
	wire [32:0]temp_sub;
	wire [32:0]temp_result_sub;

	wire [31:0] result;
	wire [32:0]temp_result3;
	wire [31:0]Result_compare;

	assign temp_and = A&B;//计算A与B的结果
	assign temp_or = A|B;//计算A或B的结果
	assign temp_sub0 = ~B;//承接B反的结果
	assign temp_xor = A^B;//计算AxorB的结果
	assign temp_sub = (ALUop[1] | ALUop[0]) ? temp_sub0 : B;//根据ALUop第2位来判断加法器实现的是减法还是加法
	assign cin = (ALUop[2] | ALUop[0]) ? 1 : 0;

	assign temp_result_sub = A + temp_sub + cin;//用加法器实现加法或是减法

	assign judge_ALU[0] = !ALUop[2];
	assign judge_ALU[1] = ALUop[2] & ALUop[0];//计算A与B的结果
	assign judge_ALU[2] = ALUop[2] & ALUop[1] & !ALUop[0];//计算A或B的结果
	assign judge_ALU[3] = ALUop[2] & !ALUop[1];//计算AxorB的结果

	assign temp_result3  =  ({33{judge_ALU[0]}}&temp_result_sub) | 
							({33{judge_ALU[1]}}&temp_and) | 
							({33{judge_ALU[2]}}&temp_or) | 
							({33{judge_ALU[3]}}&temp_xor);

	assign {CO,result} = temp_result3;//前面temp_result设置为33位，对可能的进位进行收集

	assign Overflow = (!ALUop[1] & !ALUop[0]) ? (~(A[31]^B[31]) && (A[31]^result[31])) : (~(result[31]^B[31]) && (A[31]^result[31]));
	//有符号是否有溢出判断，如果出现正加正为负或负加负为正的情况则判断为有符号加减法的溢出

	assign CarryOut = (ALUop[1] | ALUop[0]) ? (!CO) : CO ;
	//判断无符号是否由溢出，如果实行的是加法，看有没有进位就可以；如果是减法则和其取反加一相加有进位则原减法没有借位

	assign Result_compare = (!ALUop[0]) ? (result[31]^Overflow) : CarryOut;
	//判断进行的是无符号比较还是有符号比较;

	assign Result = (!ALUop[2] & ALUop[1]) ? Result_compare : result;
	//对最终结果Result进行赋值，如果需要的是加减法与或的结果，则直接将前面的result赋值给Result;如果是有符号比较大小，
	//则根据有符号是否溢出与其符号位来综合判断Result的结果

	assign Zero = Result ? 0 : 1;//根据Result是否为0判断是否将zero置零
endmodule*/

`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module ALU(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	wire CO;
	wire cin;
	wire [31:0] result;
	wire [32:0]temp_and;
	wire [32:0]temp_or;
	wire [32:0]temp_xor;
	wire [32:0]temp_nor;
	wire [32:0]temp_sub;
	wire [31:0]temp_sub0;
	wire [32:0]temp_result0;
	wire [32:0]temp_result1;
	wire [32:0]temp_result2;
	wire [32:0]temp_result3;
	wire [32:0]temp_result4;
	wire [31:0]Result_compare;

	assign temp_and = A&B;//计算A与B的结果
	assign temp_or = A|B;//计算A或B的结果
	assign temp_sub0 = ~B;//承接B反的结果
	assign temp_xor = A^B;//计算AxorB的结果
	assign temp_nor = ~(A|B);//计算AnorB的结果

	assign temp_result0 = (!ALUop[0]) ? temp_xor : temp_nor;//根据ALUop第0位判断选择nor结果还是xor结果
        assign temp_result1 = (!ALUop[0]) ? temp_and : temp_or;//根据ALUop第0位判断选择与结果还是或结果
	assign temp_result4 = (!ALUop[2]) ? temp_result1 : temp_result0;
	//根据ALUop第2位判断选择结果

	assign cin = (ALUop[2] | ALUop[0]) ? 1 : 0;
	assign temp_sub = (ALUop[2] | ALUop[0]) ? temp_sub0 : B;//根据ALUop第2位来判断加法器实现的是减法还是加法
	assign temp_result2 = A + temp_sub + cin;//用加法器实现加法或是减法
	assign temp_result3 = (!ALUop[1]) ? temp_result4 : temp_result2;//根据ALUop第1位来选择是承接加减法结果还是与或结果

	assign {CO,result} = temp_result3;//前面temp_result设置为33位，对可能的进位进行收集

	assign Overflow = (!ALUop[2]) ? (~(A[31]^B[31]) && (A[31]^result[31])) : (~(result[31]^B[31]) && (A[31]^result[31]));
	//有符号是否有溢出判断，如果出现正加正为负或负加负为正的情况则判断为有符号加减法的溢出

	assign CarryOut = (ALUop[2] | ALUop[0]) ? (!CO) : CO ;
	//判断无符号是否由溢出，如果实行的是加法，看有没有进位就可以；如果是减法则和其取反加一相加有进位则原减法没有借位

	assign Result_compare = (ALUop[2]) ? (result[31]^Overflow) : CarryOut;
	//判断进行的是无符号比较还是有符号比较;

	assign Result = (ALUop[1] & ALUop[0]) ? Result_compare : result;
	//对最终结果Result进行赋值，如果需要的是加减法与或的结果，则直接将前面的result赋值给Result;如果是有符号比较大小，
	//则根据有符号是否溢出与其符号位来综合判断Result的结果

	assign Zero = Result ? 0 : 1;//根据Result是否为0判断是否将zero置零
endmodule

