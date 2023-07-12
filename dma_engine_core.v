`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


/*module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output [31:0]       src_base,
	output [31:0]       dest_base,
	output [31:0]       tail_ptr,
	output [31:0]       head_ptr,
	output [31:0]       dma_size,
	output [31:0]       ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);
	
	// TODO: Please add your logic design here

	reg  [31:0]tail_ptr;
	reg  [31:0]head_ptr;
	reg  [31:0]dma_size;
	reg  [31:0]dest_base;
	reg  [31:0]src_base;
	reg  [31:0]ctrl_stat;

	wire [5:0]  write_engine_current_state;
	wire [2:0]  read_engine_current_state;

	wire		tail_reg_wen;
	wire  		tail_wen;
	wire		intr_reg_wen;
	wire		ctrl_wen;
	wire		dma_wen;
	wire		head_wen;
	wire		dest_wen;
	wire		src_wen;

	wire[31:0]  	tail_data;
	wire[31:0]		ctrl_data;
	wire[31:0]		dma_data;
	wire[31:0]		head_data;
	wire[31:0]		dest_data;
	wire[31:0]		src_data;


	assign	rd_req_valid = read_engine_current_state[1] & ~fifo_is_full;
	assign  rd_ready = read_engine_current_state[2];
	assign  rd_req_len = 5'd7;

	assign  wr_req_valid = write_engine_current_state[1] & ~fifo_is_empty;
	assign  wr_valid = write_engine_current_state[2];
	assign  wr_req_len = 5'd7;

	assign  fifo_wdata = rd_rdata;
	assign  fifo_wen = rd_valid & rd_ready;
	assign  fifo_rden = write_engine_current_state[4];

	assign  intr = ctrl_stat[31];
	assign	intr_reg_wen = write_engine_current_state[3];

	assign	tail_wen = reg_wr_en[2] | tail_reg_wen;
	assign  ctrl_wen = reg_wr_en[5] | intr_reg_wen;
	assign	src_wen = reg_wr_en[0];
	assign	dest_wen = reg_wr_en[1];
	assign	head_wen = reg_wr_en[3];
	assign	dma_wen = reg_wr_en[4];

	assign	tail_data   = 	(reg_wr_data & {32{reg_wr_en[2]}}) 
						| ({tail_ptr + 32'd32} & {32{tail_reg_wen}});
	assign	ctrl_data   = 	(reg_wr_data & {32{reg_wr_en[5]}}) 
						| ({1'b1, ctrl_stat[30:0]} & {32{intr_reg_wen}});
	assign	dma_data = reg_wr_data;
	assign	head_data = reg_wr_data;
	assign	dest_data = reg_wr_data;
	assign	src_data = reg_wr_data;


	always @(posedge clk) begin
		if(rst) begin
			src_base <= 32'b0;
		end
		else if(src_wen)begin
			src_base <= src_data;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			dest_base <= 32'b0;
		end
		else if(dest_wen)begin
			dest_base <= dest_data;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			tail_ptr <= 32'b0;
		end
		else if(tail_wen)begin
			tail_ptr <= tail_data;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			head_ptr <= 32'b0;
		end
		else if(head_wen)begin
			head_ptr <= head_data;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			dma_size <= 32'b0;
		end
		else if(dma_wen)begin
			dma_size <= dma_data;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst) begin
			ctrl_stat <= 32'b0;
		end
		else if(ctrl_wen)begin
			ctrl_stat <= ctrl_data;
		end
		else begin
			;
		end
	end

	read_engine_stage Read_engine_stage(
		.head_ptr(head_ptr),
		.tail_ptr(tail_ptr),
		.src_base(src_base),
		.write_engine_current_state(write_engine_current_state),
		.clk(clk),
		.rst(rst),

		.rd_req_ready(rd_req_ready),
		.rd_req_valid(rd_req_valid),

		.rd_valid(rd_valid),
		.rd_last(rd_last),
		.rd_ready(rd_ready),

		.fifo_is_full(fifo_is_full),
		.fifo_is_empty(fifo_is_empty),
		.ctrl_stat(ctrl_stat),

		.read_engine_current_state(read_engine_current_state),
		.rd_req_addr(rd_req_addr)
	);

	write_engine_stage Write_engine_stage(
		.head_ptr(head_ptr),
		.tail_ptr(tail_ptr),
		.dest_base(dest_base),
		.ctrl_stat(ctrl_stat),
		.clk(clk),
		.rst(rst),

		.fifo_rdata(fifo_rdata),

		.wr_req_valid(wr_req_valid),
		.wr_req_ready(wr_req_ready),
		.wr_valid(wr_valid),
		.wr_ready(rd_ready),

		.fifo_is_full(fifo_is_full),
		.fifo_is_empty(fifo_is_empty),

		.tail_reg_wen(tail_reg_wen),
		.wr_last(wr_last),
		.wr_data(wr_data),
		.write_engine_current_state(write_engine_current_state),
		.wr_req_addr(wr_req_addr)
	);

  
endmodule

module read_engine_stage(
	input [31:0] head_ptr,
	input [31:0] tail_ptr,
	input [31:0] src_base,
	input [5:0]  write_engine_current_state,
	input		 clk,
	input 		 rst,

	input 		rd_req_ready,
	input		rd_req_valid,

	input 		rd_valid,
	input 		rd_last,
	input		rd_ready,

	input		fifo_is_full,
	input 		fifo_is_empty,
	input[31:0] ctrl_stat,

	output [2:0] read_engine_current_state,
	output [31:0]rd_req_addr
);

	wire		RD_RD_REQ;
	wire		IDLE_RD_REQ;
	wire		RD_REQ_RD;
	wire		RD_REQ_IDLE;

	reg  [2:0] 	read_engine_current_state;
	reg [31:0] 	rd_engine_addr;

	reg [2:0] 	next_state;


	localparam  IDLE = 3'b001,
				RD_REQ = 3'b010,
				RD = 3'b100;

	assign 		RD_RD_REQ 	= rd_valid & rd_ready & rd_last;
	assign 		IDLE_RD_REQ = (write_engine_current_state[0] & ctrl_stat[0]) & (head_ptr != tail_ptr);
	assign 		RD_REQ_RD	= rd_req_valid & rd_req_ready;
	assign		RD_REQ_IDLE = fifo_is_full;

	assign 		rd_req_addr = src_base + rd_engine_addr;

	always @(posedge clk) begin
		if(rst)begin
			read_engine_current_state <= IDLE;
		end
		else begin
			read_engine_current_state <= next_state;
		end
	end

	always @(posedge clk) begin
		if(rst)begin
			rd_engine_addr <= 32'b0;
		end
		else if(RD_RD_REQ)begin
			rd_engine_addr <= rd_engine_addr + 32'd32;
		end
		else begin
			;
		end
	end

	always @(*) begin
		case(read_engine_current_state)
			IDLE:
				if(IDLE_RD_REQ)begin
					next_state = RD_REQ;
				end
				else begin
					next_state = IDLE;
				end
			RD_REQ:
				if(RD_REQ_IDLE)begin
					next_state = IDLE;
				end
				else if(RD_REQ_RD) begin
					next_state = RD;
				end
				else begin
					next_state = RD_REQ;
				end
			RD:
				if(RD_RD_REQ)begin
					next_state = RD_REQ;
				end
				else begin
					next_state = RD;
				end
			default: 
				next_state = IDLE;
		endcase
	end

endmodule

module write_engine_stage(
	input [31:0] head_ptr,
	input [31:0] tail_ptr,
	input [31:0] dest_base,
	input [31:0] ctrl_stat,
	input		 clk,
	input 		 rst,

	input[31:0]	fifo_rdata,

	input		wr_req_valid,
	input 		wr_req_ready,
	input 		wr_valid,
	input 		wr_ready,

	input 		fifo_is_full,
	input		fifo_is_empty,

	output		 tail_reg_wen,
	output		 wr_last,
	output[31:0] wr_data,
	output[5:0]  write_engine_current_state,
	output[31:0] wr_req_addr
);
	wire		write_engine_wen;
	wire		IDLE_WR_REQ;
	wire		WR_REQ_IDLE;
	wire		WR_REQ_INTR;
	wire		WR_REQ_RD_FIFO;
	wire		RD_FIFO_WR_prepare;
	wire		WR_RD_FIFO;
	wire		WR_WR_REQ;


	reg  [2:0] burst_finish_write;
	reg  [5:0] write_engine_current_state;

	reg [31:0] temp_write_data;

	reg [5:0] next_state;

	assign	write_engine_wen = write_engine_current_state[5];
	assign	IDLE_WR_REQ = ctrl_stat[0] & (head_ptr != tail_ptr) & fifo_is_full;
	assign	WR_REQ_IDLE = fifo_is_empty & ~WR_REQ_INTR;
	assign	WR_REQ_INTR = (tail_ptr[11:0] == 12'b0) & fifo_is_empty;
	assign	WR_REQ_RD_FIFO = wr_req_valid & wr_req_ready;
	assign	wr_req_addr = dest_base + tail_ptr;
	assign	RD_FIFO_WR_prepare = write_engine_current_state[4];
	assign	WR_RD_FIFO = wr_valid & wr_ready & ~wr_last;
	assign	WR_WR_REQ = wr_valid & wr_ready & wr_last;
	assign	wr_data = temp_write_data;
	assign	wr_last =  write_engine_current_state[2] & (burst_finish_write == 3'd7);
	assign  tail_reg_wen = WR_WR_REQ;

	localparam  IDLE = 6'b000001,
				WR_REQ = 6'b000010,
				WR = 6'b000100,
				Interrupt = 6'b001000,
				WR_prepare = 6'b010000,
				Read_fifo = 6'b100000;
	
	always @(posedge clk) begin
		if(write_engine_wen)begin
			temp_write_data <= fifo_rdata;
		end
		else begin
			;
		end
	end

	always @(posedge clk) begin
		if(rst)begin
			write_engine_current_state <= IDLE;
		end
		else begin
			write_engine_current_state <= next_state;
		end
	end

	always @(posedge clk) begin
		if(rst)begin
			burst_finish_write <= 3'd0;
		end
		else if(wr_valid & wr_ready)begin
			burst_finish_write <= burst_finish_write + 3'd1;
		end
		else begin
			;
		end
	end

	always @(*) begin
		case(write_engine_current_state)
			IDLE:
				if(IDLE_WR_REQ)begin
					next_state = WR_REQ;
				end
				else begin
					;
				end
			WR_REQ:
				if(WR_REQ_IDLE)begin
					next_state = IDLE;
				end
				else if(WR_REQ_INTR)begin
					next_state = Interrupt;
				end
				else if(WR_REQ_RD_FIFO) begin
					next_state = Read_fifo;
				end
				else begin
					next_state = WR_REQ;
				end
			Read_fifo:
				if(RD_FIFO_WR_prepare)begin
					next_state = WR_prepare;
				end
				else begin
					next_state = Read_fifo;
				end
			WR_prepare:
				next_state = WR;
			WR:
				if(WR_RD_FIFO)begin
					next_state = Read_fifo;
				end
				else if(WR_WR_REQ)begin
					next_state = WR_REQ;
				end
				else begin
					next_state = WR;
				end
			Interrupt:
				next_state = IDLE;
			default: 
				next_state = IDLE;
		endcase
	end

endmodule*/
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Xu Zhang (zhangxu415@mails.ucas.ac.cn)
// 
// Create Date: 06/14/2018 11:39:09 AM
// Design Name: 
// Module Name: dma_core
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module engine_core #(
	parameter integer  DATA_WIDTH       = 32
)
(
	input    clk,
	input    rst,
	
	output [31:0]       src_base,
	output [31:0]       dest_base,
	output [31:0]       tail_ptr,
	output [31:0]       head_ptr,
	output [31:0]       dma_size,
	output [31:0]       ctrl_stat,

	input  [31:0]	    reg_wr_data,
	input  [ 5:0]       reg_wr_en,
  
	output              intr,
  
	output [31:0]       rd_req_addr,
	output [ 4:0]       rd_req_len,
	output              rd_req_valid,
	
	input               rd_req_ready,
	input  [31:0]       rd_rdata,
	input               rd_last,
	input               rd_valid,
	output              rd_ready,
	
	output [31:0]       wr_req_addr,
	output [ 4:0]       wr_req_len,
	output              wr_req_valid,
	input               wr_req_ready,
	output [31:0]       wr_data,
	output              wr_valid,
	input               wr_ready,
	output              wr_last,
	
	output              fifo_rden,
	output [31:0]       fifo_wdata,
	output              fifo_wen,
	
	input  [31:0]       fifo_rdata,
	input               fifo_is_empty,
	input               fifo_is_full
);

	assign wr_req_addr        = dest_base + tail_ptr;
	assign wr_req_len         = 5'd7;
	assign wr_req_valid       = write_current_state[1] & ~fifo_is_empty;

	assign wr_data            = temp_fifo_reg;
	assign wr_valid           = write_current_state[2];

	assign wr_last            = write_current_state[2] & (wr_counter == 3'd7);

	assign fifo_rden          = write_current_state[4];

	// TODO: Please add your logic design here

//////////////////////////////////////////////////////////////////////////////////
	reg[31:0]	Src_base;
	reg[31:0]	Dest_base;
	reg[31:0]	Head_ptr;
	reg[31:0]	Tail_ptr;
	reg[31:0]	Dma_size;
	reg[31:0]	Ctrl_stat;

	wire		Src_wen;
	wire		Dest_wen;
	wire		Head_wen;
	wire		Tail_wen;
	wire		Dma_wen;
	wire		Ctrl_wen;

	wire 		intr_reg_wen, tail_reg_wen;
	wire [31:0] intr_reg_wdata, tail_reg_wdata, tail_wr_data, ctrl_wr_data;
	
	assign 		Tail_wen     = reg_wr_en[2] | tail_reg_wen;
	assign 		Ctrl_wen     = reg_wr_en[5] | intr_reg_wen;
	assign 		Src_wen      = reg_wr_en[0];
	assign 		Dest_wen     = reg_wr_en[1];
	assign 		Head_wen     = reg_wr_en[3];
	assign 		Dma_wen      = reg_wr_en[4];

	assign 		tail_wr_data = reg_wr_data & {32{reg_wr_en[2]}} | tail_reg_wdata & {32{tail_reg_wen}};
	assign 		ctrl_wr_data = reg_wr_data & {32{reg_wr_en[5]}} | intr_reg_wdata & {32{intr_reg_wen}};

	assign 		tail_reg_wen       = WR_to_WR_REQ;
	assign 		tail_reg_wdata     = tail_ptr + 32'd32;
	assign 		intr_reg_wen       = write_current_state[3];
	assign 		intr_reg_wdata     = {1'b1, ctrl_stat[30:0]}; 
///////////////////////////////////////////////////////////////////////////////

	wire  [31:0] read_engine_ptr;
	wire  [2:0]  read_current_state;


	wire  [5:0] write_current_state;
	wire  [31:0] temp_fifo_reg;
	wire  [ 2:0] wr_counter;
	wire		WR_to_WR_REQ;

///////////////////////////////////////////////////////////////////////////////
	always@(posedge clk)
	begin
		if (rst)
			Src_base <= 32'd0;
		else if (Src_wen)
			Src_base <= reg_wr_data;
		else
			Src_base <= Src_base;
	end
	assign	src_base = Src_base;

	always@(posedge clk)
	begin
		if (rst)
			Dest_base <= 32'd0;
		else if (Dest_wen)
			Dest_base <= reg_wr_data;
		else
			Dest_base <= Dest_base;
	end
	assign	dest_base = Dest_base;

	always@(posedge clk)
	begin
		if (rst)
			Head_ptr <= 32'd0;
		else if (Head_wen)
			Head_ptr <= reg_wr_data;
		else
			Head_ptr <= Head_ptr;
	end
	assign	head_ptr = Head_ptr;

	always@(posedge clk)
	begin
		if (rst)
			Tail_ptr <= 32'd0;
		else if (Tail_wen)
			Tail_ptr <= tail_wr_data;
		else
			Tail_ptr <= Tail_ptr;
	end
	assign	tail_ptr = Tail_ptr;

	always@(posedge clk)
	begin
		if (rst)
			Dma_size <= 32'd0;
		else if (Dma_wen)
			Dma_size <= reg_wr_data;
		else
			Dma_size <= Dma_size;
	end
	assign	dma_size = Dma_size;

	always@(posedge clk)
	begin
		if (rst)
			Ctrl_stat <= 32'd0;
		else if (Ctrl_wen)
			Ctrl_stat <= ctrl_wr_data;
		else
			Ctrl_stat <= Ctrl_stat;
	end
	assign	ctrl_stat = Ctrl_stat;


	assign intr = ctrl_stat[31];
/////////////////////////////////////////////////////////////////////////////////////

	read_engine_state read_engine_state0(
		.clk(clk),
		.rst(rst),
		.head_ptr(head_ptr),
		.tail_ptr(tail_ptr),
		.ctrl_stat(ctrl_stat),

		.write_current_state(write_current_state),

		.fifo_is_full(fifo_is_full),

		.rd_req_valid(rd_req_valid),
		.rd_req_ready(rd_req_ready),
		.rd_valid(rd_valid),
		.rd_ready(rd_ready),
		.rd_last(rd_last),

		.read_engine_ptr(read_engine_ptr),
		.read_current_state(read_current_state)
	);
	
	assign rd_req_valid   = read_current_state[1] & ~fifo_is_full;
	assign rd_req_addr    = src_base + read_engine_ptr;
	assign rd_req_len     = 5'd7;
	assign rd_ready       = read_current_state[2];

	assign fifo_wen       = rd_valid & rd_ready;
	assign fifo_wdata     = rd_rdata;
	assign fifo_rden      = write_current_state[4];

	write_engine_state write_engine_state0(
		.rst(rst),
		.clk(clk),

		.ctrl_stat(ctrl_stat),
		.head_ptr(head_ptr),
		.tail_ptr(tail_ptr),

		.fifo_is_full(fifo_is_full),
		.fifo_is_empty(fifo_is_empty),
		.fifo_rdata(fifo_rdata),

		.wr_req_valid(wr_req_valid),
		.wr_req_ready(wr_req_ready),
		.wr_valid(wr_valid),
		.wr_ready(wr_ready),
		.wr_last(wr_last),

		.write_current_state(write_current_state),
		.temp_fifo_reg(temp_fifo_reg),
		.wr_counter(wr_counter),
		.WR_to_WR_REQ(WR_to_WR_REQ)
	);

	assign wr_req_addr        = dest_base + tail_ptr;
	assign wr_req_len         = 5'd7;
	assign wr_req_valid       = write_current_state[1] & ~fifo_is_empty;

	assign wr_data            = temp_fifo_reg;
	assign wr_valid           = write_current_state[2];
	
	assign wr_last            = write_current_state[2] & (wr_counter == 3'd7);

endmodule

module	read_engine_state(
	input			clk,
	input			rst,

	input[31:0]		head_ptr,
	input[31:0]		tail_ptr,
	input[31:0]		ctrl_stat,

	input[5:0]		write_current_state,

	input			fifo_is_full,

	input			rd_req_valid,
	input			rd_req_ready,
	input			rd_valid,
	input			rd_ready,
	input			rd_last,

	output[31:0]	read_engine_ptr,
	output[2:0]		read_current_state
);
	localparam  IDLE    = 3'b001,
                RD_REQ  = 3'b010,
                RD      = 3'b100;

	wire        IDLE_to_RD_REQ;
	wire		RD_REQ_to_IDLE; 
	wire		RD_REQ_to_RD;
	wire		RD_to_RD_REQ;

	reg[31:0] 	read_engine_ptr;

	reg[2:0]	read_current_state;
	reg[2:0]	next_read_state;

	assign IDLE_to_RD_REQ = write_current_state[0] & ctrl_stat[0] & (head_ptr != tail_ptr);
	assign RD_REQ_to_IDLE = fifo_is_full;
	assign RD_REQ_to_RD   = rd_req_valid & rd_req_ready;
	assign RD_to_RD_REQ   = rd_valid & rd_ready & rd_last;

	always@(posedge clk)
	begin
		if (rst)
			read_engine_ptr <= 32'd0;
		else if (RD_to_RD_REQ)
			read_engine_ptr <= read_engine_ptr + 32'd32;
		else
			read_engine_ptr <= read_engine_ptr;
	end

	always@(posedge clk)
	begin
		if (rst)
			read_current_state <= IDLE;
		else
			read_current_state <= next_read_state;
	end

	always@(*)
	begin
		case (read_current_state)
			IDLE:
				if (IDLE_to_RD_REQ)
					next_read_state = RD_REQ;
				else
					next_read_state = IDLE;
			RD_REQ:
				if (RD_REQ_to_IDLE)
					next_read_state = IDLE;
				else if (RD_REQ_to_RD)
					next_read_state = RD;
				else
					next_read_state = RD_REQ;
			RD:
				if (RD_to_RD_REQ)
					next_read_state = RD_REQ;
				else
					next_read_state = RD;
			default:
					next_read_state = IDLE;
		endcase
	end
endmodule

module	write_engine_state(
	input 		rst,
	input 		clk,

	input[31:0]	ctrl_stat,
	input[31:0]	head_ptr,
	input[31:0]	tail_ptr,

	input		fifo_is_full,
	input		fifo_is_empty,
	input		fifo_rdata,

	input		wr_req_valid,
	input		wr_req_ready,
	input		wr_valid,
	input		wr_ready,
	input		wr_last,

	output[5:0]	write_current_state,
	output[31:0]temp_fifo_reg,
	output[2:0]	wr_counter,
	output		WR_to_WR_REQ
);
	localparam  IDLE    = 6'b000001,
				WR_REQ  = 6'b000010,
				WR      = 6'b000100,
				INTR    = 6'b001000,
				RD_FIFO = 6'b010000,
				WR_pre 	= 6'b100000;

	wire	IDLE_to_WR_REQ;
	wire	WR_REQ_to_IDLE;
	wire	WR_REQ_to_INTR;
	wire	WR_REQ_to_RD_FIFO;
	wire	WR_to_RD_FIFO;
	wire	WR_to_WR_REQ;
	wire	temp_wen;

	reg[31:0]	temp_fifo_reg;
	reg[2:0]	wr_counter;
	reg[5:0]	write_current_state;
	reg[5:0]	next_write_state;

	assign IDLE_to_WR_REQ     = ctrl_stat[0] & (head_ptr != tail_ptr) & fifo_is_full;
	assign WR_REQ_to_IDLE     = fifo_is_empty & !(tail_ptr[11:0] == 12'd0);
	assign WR_REQ_to_INTR     = (tail_ptr[11:0] == 12'd0) & fifo_is_empty;
	assign WR_REQ_to_RD_FIFO  = wr_req_valid & wr_req_ready;
	assign WR_to_RD_FIFO      = wr_valid & wr_ready & ~wr_last;
	assign WR_to_WR_REQ       = wr_valid & wr_ready &  wr_last;

	assign temp_wen           = write_current_state[5]; 

	always@(posedge clk)
	begin
		if (temp_wen)
			temp_fifo_reg <= fifo_rdata;
		else
			temp_fifo_reg <= temp_fifo_reg;
	end

	always@(posedge clk)
	begin
		if (rst)
			wr_counter <= 3'd0;
		else if (wr_valid & wr_ready)
			wr_counter <= wr_counter + 3'd1;
		else
			wr_counter <= wr_counter;
	end

	always@(posedge clk)
	begin
		if (rst)
			write_current_state <= IDLE;
		else
			write_current_state <= next_write_state;
	end

	always@(*)
	begin
		case (write_current_state)
			IDLE:
				if (IDLE_to_WR_REQ)
					next_write_state = WR_REQ;
				else
					next_write_state = IDLE;
			WR_REQ:
				if (WR_REQ_to_INTR)
					next_write_state = INTR;
				else if (WR_REQ_to_RD_FIFO)
					next_write_state = RD_FIFO;
				else if (WR_REQ_to_IDLE)
					next_write_state = IDLE;
				else
					next_write_state = WR_REQ;
			RD_FIFO:
				next_write_state = WR_pre;
			WR_pre:
				next_write_state = WR;
			WR:
				if (WR_to_RD_FIFO)
					next_write_state = RD_FIFO;
				else if (WR_to_WR_REQ)
					next_write_state = WR_REQ;
				else
					next_write_state = WR;
			INTR:
				next_write_state = IDLE;
			default:
				next_write_state = IDLE;
		endcase
	end

endmodule

