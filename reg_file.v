`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	reg [31:0] rf [31:0];//定义32个32位寄存器

	always @(posedge clk) begin
		if(wen == 1 && waddr != 0)begin
			rf[waddr] <= wdata;
		end
    end
	//在每个时钟的上升沿，如果写使能信号和写地址均为有效值（写地址不为0），
	//则将要写入的数据写入该地址对应的寄存器中。


	assign rdata1 = (raddr1) ? rf[raddr1] : 0;
	assign rdata2 = (raddr2) ? rf[raddr2] : 0;
	//异步读。如果要读出数据，且读地址不为0，则读出该地址对应的寄存器中的数据；
	//如果读地址位0，则直接将读出结果赋值位0就可以。

	// TODO: Please add your logic design here
	
endmodule
