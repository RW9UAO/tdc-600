`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:16:30 03/30/2023 
// Design Name: 
// Module Name:    fdc_ram 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module fdc_ram(
	// SRAM chip
	output wire [18:0]SRAM_Addr,
	inout wire [7:0]SRAM_Data,
	output wire SRAM_OE,
	output wire SRAM_WE,
	output wire SRAM_CS,
	//----------------------------
	// MSX connection
	input wire [15:0]MSX_A,
	inout wire [7:0]MSX_D,
	input wire MSX_CLK,
	input wire MSX_nWR,
	input wire MSX_nRD,
	input wire MSX_nSLTSL,
	input wire MSX_nCS1,
	//----------------------------
	// STM32 connection
	input wire 	SPI_CS,
	input wire 	SPI_MOSI,
	output wire	SPI_MISO,
	input wire 	SPI_CLK,
	input wire 	RAM_LOAD,	// stm32 load SRAM
	input wire 	RAM_nWR,		// stm32 /WR impulse
//	input wire	MAPPER,		// change floppy to mapper mode
//	input wire	[1:0]MAPPER_TYPE,
	//----------------------------
	output wire nFDC_CS		// WD37C65C /CS
	
    );



	wire nLDOR;
	wire sel1;
	wire sel2;
	reg [7:0]WD_stat = 8'h80;
	reg [7:0]WD_data = 8'h00;

	reg [26:0]spi_dataW;
	reg [15:0]spi_dataR;
	reg nwdcs;
//========================================================================

	
	wire [18:13]MA;
	reg [4:0]HC670_0;
	reg [4:0]HC670_1;
	reg [4:0]HC670_2;
	reg [4:0]HC670_3;
	wire nHC670WR;
	wire [1:0]HC670AB;
	
	
	reg MAPPER;
	reg [1:0]MAPPER_TYPE;


	assign sel1   = ( MSX_A[14] == 1'b0 && MSX_nSLTSL == 1'b0 ) ? 1'b1 : 1'b0;
	assign sel2   = ( nwdcs == 1'b0 && MSX_nRD == 1'b0 ) ? 1'b1 : 1'b0;
	assign nLDOR  = ( MSX_A[13:12] == 2'b01 && sel1 ) ? 1'b0 : 1'b1;
	assign MSX_D  = 
// mapper, need to be removed in ROM only version
						( ( MAPPER == 1'b1 && RAM_LOAD == 1'b0 && SRAM_CS == 1'b0 && SRAM_OE == 1'b0 ) || 
// TDC-600, need to be removed in ROM only version
						( MSX_nCS1 == 1'b0 && MSX_nSLTSL == 1'b0 && RAM_LOAD == 1'b0 ) ) ? SRAM_Data :
						( sel2 && MSX_A[0] == 1'b0 )	? WD_stat :
						( sel2 && MSX_A[0] == 1'b1 )	? WD_data : 8'bZZZZZZZ;

	assign sel3 = 
//	konami
						( MAPPER_TYPE == 2'b10 ) ? MSX_A[12] :
//	konami SCC
						(MSX_A[11] || (!MSX_A[12])) ? 1'b1 : 1'b0;

	assign nHC670WR = ( sel3 || ( !( MSX_nWR == 1'b0 && MSX_nSLTSL == 1'b0 ) ))? 1'b1 : 1'b0;


	assign HC670AB[1:0] = {MSX_A[15], MSX_A[13]};
	assign MA[18] = 1'b0;
	assign MA[17:13] = ( HC670AB == 2'b00 ) ? HC670_0 :
					( HC670AB == 2'b01) ? HC670_1 :
					( HC670AB == 2'b10) ? HC670_2 : HC670_3;

	always@( negedge MSX_CLK )begin
		if( nHC670WR == 1'b0 )begin
			case( HC670AB )
				2'b00:begin
					HC670_0 <= MSX_D[4:0];
					end
				2'b01:begin
					HC670_1 <= MSX_D[4:0];
					end
				2'b10:begin
					HC670_2 <= MSX_D[4:0];
					end
				2'b11:begin
					HC670_3 <= MSX_D[4:0];
					end
			endcase
		end else if( MAPPER == 1'b0 )begin
			HC670_0 <= 5'b00000 ;
			HC670_1 <= 5'b00000 ;
			HC670_2 <= 5'b00000 ;
			HC670_3 <= 5'b00000 ;
		end
	end


//========================================================================
// 32 bit, 2 packet contain SRAM addr and SRAM data
	assign SRAM_Addr 	= 
// TDC-600
	( RAM_LOAD ) ? spi_dataW[26:8] : 
	( MAPPER == 1'b0 ) ? {5'b00000, MSX_A[13:0]} :
// mapper
	{MA[18:13], MSX_A[12:0]};
	assign SRAM_Data	= ( RAM_LOAD ) ? spi_dataW[7:0] : 8'bZZZZZZZ;
	assign SRAM_OE 	= ( RAM_LOAD ) ? 1'b1 : MSX_nSLTSL;
	assign SRAM_CS 	= 
// TDC-600
	( RAM_LOAD ) ? RAM_nWR :
	( MAPPER == 1'b0 ) ? MSX_nCS1 :
// mapper
	MSX_nRD;									
	assign SRAM_WE 	= ( RAM_LOAD ) ? RAM_nWR : 1'b1;
//========================================================================
	assign nFDC_CS = nwdcs;

reg [5:0]cnt;
	// SPI output data multiplexor
	assign SPI_MISO = (cnt == 0  ) ? spi_dataR[15] : // do not change to 1'b1
							(cnt == 1  ) ? spi_dataR[14] :
							(cnt == 2  ) ? spi_dataR[13] :
							(cnt == 3  ) ? spi_dataR[12] :
							(cnt == 4  ) ? spi_dataR[11] :
							(cnt == 5  ) ? spi_dataR[10] : // do not change to 1'b0
							(cnt == 6  ) ? spi_dataR[9] :  // do not change to 1'b0
							(cnt == 7  ) ? spi_dataR[8] :  // do not change to 1'b0
							(cnt == 8  ) ? spi_dataR[7] :
							(cnt == 9  ) ? spi_dataR[6] :
							(cnt == 10 ) ? spi_dataR[5] :
							(cnt == 11 ) ? spi_dataR[4] :
							(cnt == 12 ) ? spi_dataR[3] :
							(cnt == 13 ) ? spi_dataR[2] :
							(cnt == 14 ) ? spi_dataR[1] :
							spi_dataR[0];


always @( posedge SPI_CLK )begin
		// SPI input register
		spi_dataW <= {spi_dataW[25:0], SPI_MOSI};

		//spimiso <= spi_dataR[15];
		//spi_dataR <= {spi_dataR[14:0], 1'b1};
		// ^^^ ERROR - multiple driving
		// spi_dataR need to be in one clock block
end

// SPI counter
always @( negedge SPI_CLK or posedge SPI_CS )begin
	if( SPI_CS == 1'b1 )begin
		cnt <= 1'b0;		// reset counter on SPI transfer end
	end else begin
		cnt <= cnt + 1'b1;	// inc counter
	end
end

// assing method generate many short glitches
// need sync on CPU clock
always@( posedge MSX_CLK )begin
	if( MSX_A[13:12] == 2'b00 && sel1 )begin
		nwdcs <= 1'b0;
	end else begin
		nwdcs <= 1'b1;
	end
end

// latch data to SPI reg
always@( negedge MSX_CLK )begin
	// check for already sending SPI
	if( SPI_CS == 1'b1 && ( nwdcs == 1'b0 || nLDOR == 1'b0 ) )begin
				spi_dataR[15] <= 1'b1;		// header flag, not used
				spi_dataR[14] <= nLDOR;
				spi_dataR[13] <= MSX_A[0];
				spi_dataR[12] <= MSX_nWR;
				spi_dataR[11] <= MSX_nRD;
				spi_dataR[10:8] <= 3'b000;	// free bits
			
				// MSX write WD37C65C data or LDOR reg 
				spi_dataR[7:0] <= MSX_D[7:0];
	end
end

// end of SPI transfer
always @( posedge SPI_CS )begin
	if( RAM_LOAD == 1'b0 )begin
		// 16 bit, 1 packet
		// 0x80xx - write to status reg
		// 0x40xx - write to data reg
		// 0x20xx - enable mapper mode
		//		00 - ASCII8
		//		01 - ASCII16
		//		10 - konami
		//		11 - konami SCC
		if( spi_dataW[15] == 1'b1 )begin
			WD_stat <= spi_dataW[7:0];
		end else if( spi_dataW[14] == 1'b1 )begin
			WD_data <= spi_dataW[7:0];
		end else if( spi_dataW[13] == 1'b1 )begin
			MAPPER <= 1'b1;
			MAPPER_TYPE <= spi_dataW[1:0];
		end else begin
			MAPPER <= 1'b0;
		end
	end
end
//========================================================================

endmodule

