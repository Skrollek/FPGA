//todo check if two ports are needed, maybe use two 8 bit memories instead
 module Chip8_memory(
		input  logic	    clk,
		input  logic 		wren,
		input  logic [11:0] write_addr,
		input  logic [7:0]  write_data,
		input  logic        reen,
		input  logic [11:0] read_addr,
		output logic [7:0]  read_data,
		output logic        read_ack);
		
		logic[4095:0][7:0] mem;
		
		initial $readmemh("font.hex", data, 'h030, 'h07f);
		initial $readmemh("random.hex", data, 'h200, 'hfff);
		always_ff @(posedge clk) begin
			read_ack <= 0;
			if(wren) mem[write_addr] <= writedata;
			if(reen) begin 
				read_data <= mem[read_addr];
				read_ack  <= 1;
			end
		end

endmodule 