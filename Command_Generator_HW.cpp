//#include "header.h"
#include <hls_stream.h>
#include <ap_int.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <hls_math.h>

// bank mode register enable or disable
#define ENABLE_REGISTER 	1
#define DISABLE_REGISTER 	0

// AAM mode enable
#define AAM_DISABLE 0

// splitEngine isjump
#define IS_JUMP 	1
#define NOT_JUMP 	0

// bank number
#define BANK_NUM 10

// bank mode
#define SINGLE_BANK 	1
#define ALL_BANK 		2
#define ALL_BANK_PIM 	3

// crf or bank address register
#define CRF_REG 		0
#define BANK_ADDR_REG 	1

// grf_a_b
#define DATA_GRF_A	1
#define DATA_GRF_B	2

// register - source, destination
#define BANK		0b000
#define GRF_A 	0b001
#define GRF_B 	0b010
#define SRF_A 	0b011
#define SRF_M		0b100

// register local_address
#define SBMR_LOCAL_ADDR			0x3fff
#define ABMR_LOCAL_ADDR			0x3ffe
#define PIM_OP_MODE_LOCAL_ADDR 	0x3ffd
#define CRF_LOCAL_ADDR			0x3ffc
#define GRF_A_B_LOCAL_ADDR		0x3ffb
#define SRF_A_M_LOCAL_ADDR		0x3ffa
#define BANK_ADDR_REG_ADDR		0x3ff9

// opcode - control
#define NOP 	0b0000
#define JUMP 	0b0001
#define EXIT	0b0010
// opcode - move data
#define MOV 	0b0100
#define FILL 	0b0101
// opcode - arithmetic
#define ADD	 	0b1000
#define MUL 	0b1001
#define MAC 	0b1010
#define MAD 	0b1011

#define READ 	0
#define WRITE 	1

#define REAL_ALL_BANK 	0
#define SINGLE_ALL_BANK 1

#define COMPUTE_UNIT_NUM 16

// register for loop num
#define GRF_NUM 8
#define SRF_NUM 4

// bit information
#define COLUMN_BIT		5
#define ROW_BIT			19
#define BANK_GROUP_BIT	2
#define BANK_ADDR_BIT	2
#define CHANNEL_BIT		4
#define GRF_A_IDX_BIT	3
#define GRF_B_IDX_BIT	3
#define ROW_BIT_AAM		18
#define AAM_BIT			1
#define OPCODE_BIT		4
#define SRC_1_NUM_BIT	3
#define SRC_2_NUM_BIT	3
#define SRC_1_BIT			3
#define DEST_BIT			3
#define IMM_1_BIT			11
#define IMM_0_BIT			7
#define IMM_0_MSB_BIT	1
#define SRC_0_NUM_BIT	3
#define DEST_NUM_BIT		3
#define R_BIT				1
#define SRC_0_BIT			3
#define SRC_DEST_BIT		3

#define FLAG_ON 	1
#define FLAG_OFF	0

#define ADD_OP	1
#define BN_OP		2
#define GEMV_OP	3
#define LSTM_OP	4

#define IS_FINISH 	1
#define NOT_FINISH	0

#define MAX_CHANNEL				16
#define BANK_NUM_PER_CHANNEL	16

#define TRUE	1
#define FALSE	0

#define FILL_NUM	7
#define AAM_NUM		7

#define BANK_NUM_PER_CHANNEL	16

#define FIRST_META_DATA_OFFSET	0
#define SECOND_META_DATA_OFFSET	69001
#define THIRD_META_DATA_OFFSET	69106
#define FORTH_META_DATA_OFFSET	69211
#define FIFTH_META_DATA_OFFSET	69316
#define SIXTH_META_DATA_OFFSET	69421

typedef ap_uint<32> stm_b;
typedef ap_uint<16> src_b;
typedef ap_uint<32> interface_b;
typedef ap_uint<32> cmd_b;
typedef ap_uint<32> addr_b;

//typedef half half_fp;

//static uint32_t crf_reg[MAX_CHANNEL*BANK_NUM_PER_CHANNEL][32];
//static uint16_t grf_a_reg[MAX_CHANNEL*BANK_NUM_PER_CHANNEL][8][16];
//static uint16_t grf_b_reg[MAX_CHANNEL*BANK_NUM_PER_CHANNEL][8][16];
//static uint16_t srf_a_reg[MAX_CHANNEL*BANK_NUM_PER_CHANNEL][8];
//static uint16_t srf_m_reg[MAX_CHANNEL*BANK_NUM_PER_CHANNEL][8];

static uint32_t crf_reg[MAX_CHANNEL][32];
static uint16_t grf_a_reg[MAX_CHANNEL][8][16];
static uint16_t grf_b_reg[MAX_CHANNEL][8][16];
static uint16_t srf_a_reg[MAX_CHANNEL][8];
static uint16_t srf_m_reg[MAX_CHANNEL][8];

bool exit_reg[MAX_CHANNEL];
bool pim_op_mode[MAX_CHANNEL];

int test_val;
uint16_t token_dim;

bool is_store[MAX_CHANNEL];

uint8_t fill_count[MAX_CHANNEL];
uint8_t aam_count[MAX_CHANNEL];

uint32_t last_data_size;

// additional info
uint16_t n_iter[MAX_CHANNEL];
//uint16_t n_cmdgroup;
uint16_t n_cmd_group[MAX_CHANNEL];
uint32_t operand_addr[MAX_CHANNEL][16];
uint32_t data_addr[MAX_CHANNEL][16];

// command info
uint8_t op_code_tg[MAX_CHANNEL][16];
uint8_t addr_tg[MAX_CHANNEL][16];
uint8_t data_tg[MAX_CHANNEL][16];
uint16_t addr_tg_step[MAX_CHANNEL][16];
uint16_t data_tg_step[MAX_CHANNEL][16];
uint16_t n_cmd[MAX_CHANNEL][16];

// address generator register
uint8_t pim_op_reg[MAX_CHANNEL][16];
uint32_t pim_reg[MAX_CHANNEL][16];

uint32_t operand_addr_reg[MAX_CHANNEL][16];
uint32_t data_addr_reg[MAX_CHANNEL][16];
uint32_t command_gen_data[256][256];

union ammAddrBit{
	struct{
		uint32_t 	 		: 4;
		uint32_t  			: 2;
		uint32_t  			: 2;
		uint32_t grf_a_idx 	: GRF_A_IDX_BIT;
		uint32_t grf_b_idx 	: GRF_B_IDX_BIT;
		uint32_t 		: ROW_BIT_AAM;
	};
	uint32_t aam_addr_bit;
};

union checkBit{
	struct{
		uint32_t  			: 15;
		uint32_t aam		: AAM_BIT;
		uint32_t  			: 12;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t check_instruction;
};

union controlBit{
	struct{
		uint32_t imm_1 		: IMM_1_BIT;
		uint32_t imm_0 		: IMM_0_BIT;
		uint32_t imm_0_msb 	: IMM_0_MSB_BIT;
		uint32_t  			: 9;
		uint32_t opcode	 	: OPCODE_BIT;
	};
	unsigned int contorl_instrcution;
};

union dataBit{
	struct{
		uint32_t  			: 4;
		uint32_t src_0_num 	: SRC_0_NUM_BIT;
		uint32_t  			: 1;
		uint32_t dest_num 	: DEST_NUM_BIT;
		uint32_t  			: 1;
		uint32_t r 			: R_BIT;
		uint32_t  			: 2;
		uint32_t aam		: AAM_BIT;
		uint32_t  			: 6;
		uint32_t src_0 		: SRC_0_BIT;
		uint32_t dest	 	: SRC_DEST_BIT;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t data_instrcution;
};

union aluBit{
	struct{
		uint32_t src_1_num 	: SRC_1_NUM_BIT;
		uint32_t  			: 1;
		uint32_t src_0_num	: SRC_0_NUM_BIT;
		uint32_t 	 		: 1;
		uint32_t dest_num 	: DEST_NUM_BIT;
		uint32_t  			: 4;
		uint32_t aam		: AAM_BIT;
		uint32_t src_2_num 	: SRC_2_NUM_BIT;
		uint32_t src_1 		: SRC_1_BIT;
		uint32_t src_0 		: SRC_0_BIT;
		uint32_t dest 		: DEST_BIT;
		uint32_t opcode 	: OPCODE_BIT;
	};
	uint32_t alu_instruction;
};

union addrBit{
	struct{
		uint32_t ch 	: CHANNEL_BIT;
		uint32_t ba 	: BANK_ADDR_BIT;
		uint32_t bg 	: BANK_GROUP_BIT;
		uint32_t col 	: COLUMN_BIT;
		uint32_t row 	: ROW_BIT;
	};
	uint32_t addr;
};

union bitSplit{
	struct{
		uint32_t byte_0 : 16;
		uint32_t byte_1 : 16;
	};
	uint32_t full_byte;
};

union opcodeSplit{
	struct{
		uint32_t opcode : 4;
		uint32_t is_count : 28;
	};
	uint32_t full_info;
};

typedef struct{
	ap_uint<16> operand_0[16];
	ap_uint<16> operand_1[16];
	ap_uint<16> destination_source[16];
	ap_uint<16> mad_srf_a_operand[16];
	ap_uint<16> mad_srf_m_operand[16];

	ap_uint<16> execution_result[16];
}operandSource;

typedef struct{
	ap_uint<32> instruction_reg;
	ap_uint<32> bank_addr_reg;

	ap_uint<4> all_bank_pim_opcode;

	ap_uint<3> operand_source_0;
	ap_uint<3> operand_source_1;
	ap_uint<3> operand_destination;

	ap_uint<3> operand_source_0_num;
	ap_uint<3> operand_source_1_num;
	ap_uint<3> operand_destination_num;
	ap_uint<3> srf_source_num;

	ap_uint<1> aam_mode;

	ap_uint<7> imm_0;
	ap_uint<11> imm_1;
	ap_uint<1> imm_0_msb;

	ap_uint<3> grf_a_modified_idx;
	ap_uint<3> grf_b_modified_idx;
}operandInfo;

typedef struct{
	ap_uint<6> sob;
	ap_uint<6> eob;
	ap_uint<1> jump_flag;

	ap_uint<11> body_size;
	ap_uint<32> loop_size;
	ap_uint<6> jump_program_count;
}jumpInfo;

union quadToSingle{
	struct{
		uint32_t first		: 8;
		uint32_t second		: 8;
		uint32_t third		: 8;
		uint32_t forth  	: 8;
	};
	uint32_t split_byte_single;
};

union quadToDouble{
	struct{
		uint32_t first		: 16;
		uint32_t second		: 16;
	};
	uint32_t split_byte_double;
};

union toggleBit{
    struct{
        ap_uint<19> first  :   11;
        ap_uint<19> toggle :   1;
        ap_uint<19> last   :   7;
    };
    ap_uint<19> row_bit;
};

//memToStream(packed_data, 128, strm_0, 0); // memory to stream
//streamToMem(channel_0, 128, strm_0, 0);

void memToStream(
		uint32_t* in_r, ap_uint<32> n,
		hls::stream<uint32_t> & strm, addr_b offset){
	for(uint32_t i = offset; i < (n + offset); i++){
#pragma HLS PIPELINE
		strm << in_r[i];
	}
}

void streamToMem(uint32_t* out_r, uint32_t n, hls::stream<uint32_t> & strm, uint32_t offset){
	for(uint32_t i = offset; i < (n+offset); i++){
#pragma HLS PIPELINE
		strm >> out_r[i];
	}
}

void getOperand(
		ap_uint<3> src_bit, ap_uint<3> src_bit_num,
		hls::stream<uint32_t> & strm, uint32_t offset,
		src_b *data_from_source, uint32_t *channel, uint32_t channel_idx){
//#pragma HLS INLINE off
#pragma HLS INLINE
	bitSplit bs;

	switch(src_bit){
		case GRF_A:{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
				data_from_source[i] = grf_a_reg[channel_idx][src_bit_num][i];
			}
			break;
		}
		case GRF_B:{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
				data_from_source[i] = grf_b_reg[channel_idx][src_bit_num][i];
			}
			break;
		}
		case SRF_A :{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
				data_from_source[i] = srf_a_reg[channel_idx][src_bit_num];
			}
			break;
		}
		case SRF_M :{
			for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
				data_from_source[i] = srf_m_reg[channel_idx][src_bit_num];
			}
			break;
		}
		case BANK:{
			uint32_t tmp[8];
			memToStream(channel, 8, strm, offset); // memory to stream
			streamToMem(channel, 8, strm, offset);

			for(int i = 0; i < (COMPUTE_UNIT_NUM/2); i++){
#pragma HLS unroll
				bs.full_byte = tmp[i];
				data_from_source[2*i] = bs.byte_0;
				data_from_source[2*i+1] = bs.byte_1;
			}
			break;
		}
	}
}

void writeResult(ap_uint<3> destination, ap_uint<3> destination_num,
		src_b *execution_result, uint32_t channel_idx, uint32_t addr,
		hls::stream<uint32_t> & strm, uint32_t *channel){

	switch(destination){
		case GRF_A:{
			for(int i = 0; i < GRF_NUM; i++){
#pragma HLS unroll
				grf_a_reg[channel_idx][destination_num][i] = execution_result[i];
			}
			break;
		}
		case GRF_B:{
			for(int i = 0; i < GRF_NUM; i++){
#pragma HLS unroll
				grf_b_reg[channel_idx][destination_num][i] = execution_result[i];
			}
			break;
		}
		case BANK:{
			for(int i = 0; i < GRF_NUM; i++){
#pragma HLS unroll
				memToStream(channel, 8, strm, addr); // memory to stream
				streamToMem(channel, 8, strm, addr);
			}
			break;
		}
	}
}

void intToShort(uint32_t *data_from_host, uint8_t idx, uint8_t a, uint32_t channel_idx){
	bitSplit bs;

	if(a == DATA_GRF_A){
		for(int i = 0; i < GRF_NUM; i++){
#pragma HLS unroll
			bs.full_byte = data_from_host[i];
			grf_a_reg[channel_idx][idx][2*i] = bs.byte_0;
			grf_a_reg[channel_idx][idx][2*i+1] = bs.byte_1;
		}
	}else if(a == DATA_GRF_B){
		for(int i = 0; i < GRF_NUM; i++){
#pragma HLS unroll
			bs.full_byte = data_from_host[i];
			grf_b_reg[channel_idx][idx][2*i] = bs.byte_0;
			grf_b_reg[channel_idx][idx][2*i+1] = bs.byte_1;
		}
	}else{
		for(int i = 0; i < 4; i++){
#pragma HLS unroll
			bs.full_byte = data_from_host[i];
			srf_a_reg[channel_idx][2*i] = bs.byte_0;
			srf_a_reg[channel_idx][2*i+1] = bs.byte_1;
		}

		for(int i = 0; i < 4; i++){
#pragma HLS unroll
			bs.full_byte = data_from_host[i+4];
			srf_m_reg[channel_idx][2*i] = bs.byte_0;
			srf_m_reg[channel_idx][2*i+1] = bs.byte_1;

		}
	}
}

void dataToReg(uint32_t *data_from_host, uint8_t offset, bool reg, uint32_t channel_idx){

	for(int i = 0; i < 8; i++){
#pragma HLS unroll
		crf_reg[channel_idx][i + offset] = data_from_host[i];
	}
}

void writeDataToGrf(
		uint8_t column_local_address, uint32_t *data_from_host, uint32_t channel_idx){
	switch(column_local_address){
		case 0:{ // write data to grf_a[0]
			intToShort(data_from_host, 0, DATA_GRF_A, channel_idx);
			break;
		}
		case 1:{ // write data to grf_a[1]
			intToShort(data_from_host, 1, DATA_GRF_A, channel_idx);
			break;
		}
		case 2:{ // write data to grf_a[2]
			intToShort(data_from_host, 2, DATA_GRF_A, channel_idx);
			break;
		}
		case 3:{ // write data to grf_a[3]
			intToShort(data_from_host, 3, DATA_GRF_A, channel_idx);
			break;
		}
		case 4:{ // write data to grf_a[4]
			intToShort(data_from_host, 4, DATA_GRF_A, channel_idx);
			break;
		}
		case 5:{ // write data to grf_a[5]
			intToShort(data_from_host, 5, DATA_GRF_A, channel_idx);
			break;
		}
		case 6:{ // write data to grf_a[6]
			intToShort(data_from_host, 6, DATA_GRF_A, channel_idx);
			break;
		}
		case 7:{ // write data to grf_a[7]
			intToShort(data_from_host, 7, DATA_GRF_A, channel_idx);
			break;
		}
		case 8:{ // write data to grf_b[0]
			intToShort(data_from_host, 0, DATA_GRF_B, channel_idx);
			break;
		}
		case 9:{ // write data to grf_b[1]
			intToShort(data_from_host, 1, DATA_GRF_B, channel_idx);
			break;
		}
		case 10:{ // write data to grf_b[2]
			intToShort(data_from_host, 2, DATA_GRF_B, channel_idx);
			break;
		}
		case 11:{ // write data to grf_b[3]
			intToShort(data_from_host, 3, DATA_GRF_B, channel_idx);
			break;
		}
		case 12:{ // write data to grf_b[4]
			intToShort(data_from_host, 4, DATA_GRF_B, channel_idx);
			break;
		}
		case 13:{ // write data to grf_b[5]
			intToShort(data_from_host, 5, DATA_GRF_B, channel_idx);
			break;
		}
		case 14:{ // write data to grf_b[6]
			intToShort(data_from_host, 6, DATA_GRF_B, channel_idx);
			break;
		}
		case 15:{ // write data to grf_b[7]
			intToShort(data_from_host, 7, DATA_GRF_B, channel_idx);
			break;
		}
	}
}

void writeDataToCrf(
		uint8_t column_local_address, uint32_t *data_from_host, uint32_t channel_idx){
	switch(column_local_address){
		case 0:{ // write data to crf[0] ~ crf[7]
			dataToReg(data_from_host, 0, CRF_REG, channel_idx);
			break;
		}
		case 1:{ // write data to crf[8] ~ crf[15]
			dataToReg(data_from_host, 8, CRF_REG, channel_idx);
			break;
		}
		case 2:{ // write data to crf[16] ~ crf[23]
			dataToReg(data_from_host, 16, CRF_REG, channel_idx);
			break;
		}
		case 3:{ // write data to crf[24] ~ crf[31]
			dataToReg(data_from_host, 24, CRF_REG, channel_idx);
			break;
		}
	}
}

void writeDataToSfr(uint32_t *data_from_host, uint32_t channel_idx){
	intToShort(data_from_host, 0, 0, channel_idx);
}

void addUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result){
//	half_fp tmp_0;
//	half_fp tmp_1;

	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a+b;
#pragma HLS unroll
//		tmp_0 = static_cast<half_fp>(operand_0[i]);
//		tmp_1 = static_cast<half_fp>(operand_1[i]);
//
//		execution_result[i] = static_cast<src_b>(tmp_0 + tmp_1);
		execution_result[i] = operand_0[i] + operand_1[i];
	}
}

void mulUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result, uint16_t dim){
//	half_fp tmp_0;
//	half_fp tmp_1;

	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
#pragma HLS unroll
//		tmp_0 = static_cast<half_fp>(operand_0[i]);
//		tmp_1 = static_cast<half_fp>(operand_1[i]);
//		execution_result[i] = static_cast<src_b>(tmp_0 * tmp_1);

		execution_result[i] = operand_0[i] * operand_1[i] / dim;
	}
}

void macUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result, src_b *destination_source){
//	half_fp tmp_0;
//	half_fp tmp_1;
//	half_fp tmp_2;

	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
#pragma HLS unroll
//		tmp_0 = static_cast<half_fp>(operand_0[i]);
//		tmp_1 = static_cast<half_fp>(operand_1[i]);
//		tmp_2 = static_cast<half_fp>(destination_source[i]);

//		execution_result[i] = static_cast<src_b>((tmp_0 * tmp_1) + tmp_2);

		execution_result[i] = (operand_0[i] * operand_1[i]) + destination_source[i];
	}
}

void madUnit(src_b *operand_0, src_b *operand_1, src_b *execution_result){
//	half_fp tmp_0;
//	half_fp tmp_1;
//	half_fp tmp_2 = static_cast<half_fp>(srf_a_reg[0]);

	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){ //	result = a*b;
#pragma HLS unroll
//		tmp_0 = static_cast<half_fp>(operand_0[i]);
//		tmp_1 = static_cast<half_fp>(operand_1[i]);

//		execution_result[i] = static_cast<src_b>((tmp_0 * tmp_1) + tmp_2);
		execution_result[i] = (operand_0[i] * operand_1[i]) + srf_a_reg[0];
	}
}

void expUnit(uint16_t *operand_0, uint16_t *execution_result){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
		execution_result[i] = (uint16_t)exp(operand_0[i]);
	}
}

void reduceUnit(uint16_t *operand_0, uint16_t *execution_result){
	for(int i = 1; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
		operand_0[0] += operand_0[i];
	}
	execution_result[0] = operand_0[0];
}

void divUnit(uint16_t *operand_0, uint16_t *operand_1, uint16_t execution_result){
	for(int i = 0; i < COMPUTE_UNIT_NUM; i++){
#pragma HLS unroll
		execution_result[i] = operand_0[i] / operand_1[0];
	}
}

void loadEngine(
		operandInfo *op_info_ptr, operandSource *op_source_ptr, uint32_t addr,
		hls::stream<uint32_t> & strm, uint32_t *channel, uint32_t channel_idx){
//#pragma HLS INLINE off
#pragma HLS INLINE

	if((op_info_ptr->all_bank_pim_opcode == MOV) || (op_info_ptr->all_bank_pim_opcode == FILL)){
		// fill operand data to variable
		getOperand(op_info_ptr->operand_source_0, op_info_ptr->operand_source_0_num,
						strm, addr, (*op_source_ptr).operand_0, channel,
						channel_idx);
	}else if((op_info_ptr->all_bank_pim_opcode == ADD) ||
			(op_info_ptr->all_bank_pim_opcode == MUL) ||
			(op_info_ptr->all_bank_pim_opcode == MAC) ||
			(op_info_ptr->all_bank_pim_opcode == MAD)){
		// fill operand data to variable - srf number?
		getOperand(op_info_ptr->operand_source_0, op_info_ptr->operand_source_0_num,
				strm, addr, (*op_source_ptr).operand_0, channel,
				channel_idx);
		getOperand(op_info_ptr->operand_source_1, op_info_ptr->operand_source_1_num,
				strm, addr, (*op_source_ptr).operand_1, channel,
				channel_idx);
		getOperand(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
				strm, addr, (*op_source_ptr).destination_source, channel,
				channel_idx);

		getOperand(SRF_A, op_info_ptr->srf_source_num,
				strm, addr, (*op_source_ptr).mad_srf_a_operand, channel,
				channel_idx);
	}
}

void fetchInstruction(operandInfo *op_info_ptr, ap_uint<6> *program_count, uint32_t channel_idx){
	(*op_info_ptr).instruction_reg = crf_reg[channel_idx][program_count[channel_idx]];
	program_count[channel_idx] = program_count[channel_idx] + 1;
}

void jumpEngine(
        operandInfo *op_info_ptr, jumpInfo *jump_info_ptr,
		ap_uint<32> *jump_counter, ap_uint<6> *jump_program_count){
    (*jump_info_ptr).jump_program_count = jump_info_ptr->sob;
    (*jump_info_ptr).body_size = jump_info_ptr->eob - jump_info_ptr->sob + 1;
    (*jump_info_ptr).loop_size = jump_info_ptr->body_size * op_info_ptr->imm_1;
	(*jump_info_ptr).jump_flag = FLAG_ON;
}

void decodingEngine(operandInfo *op_info_ptr, jumpInfo *jump_info_ptr, uint32_t addr,
		ap_uint<6> *program_count, ap_uint<32> *jump_counter, ap_uint<6> *jump_program_count,
		uint32_t channel_idx){
#pragma HLS INLINE off

	aluBit alu;
	dataBit data;
	controlBit control;
	checkBit check;
	ammAddrBit amm_addr;

	fetchInstruction(op_info_ptr, program_count, channel_idx);

	check.check_instruction = op_info_ptr->instruction_reg;
	(*op_info_ptr).all_bank_pim_opcode = check.opcode; // opcode

	(*op_info_ptr).bank_addr_reg = addr;

	if(check.opcode == JUMP){
		control.contorl_instrcution = op_info_ptr->instruction_reg;
		(*op_info_ptr).imm_1 = control.imm_1;

		jump_counter[channel_idx] = control.imm_1;

		(*jump_info_ptr).eob = program_count[channel_idx] - 2;

		if(control.imm_0_msb){
			(*jump_info_ptr).sob = program_count[channel_idx] - control.imm_0 - 1;
		}else{
			(*jump_info_ptr).sob = program_count[channel_idx] + control.imm_0 - 1;
		}

		jumpEngine(op_info_ptr, jump_info_ptr, jump_counter, jump_program_count);
	}

	if(check.opcode == MOV){
		data.data_instrcution = op_info_ptr->instruction_reg;
		amm_addr.aam_addr_bit = op_info_ptr->bank_addr_reg;

		// source and destination
		(*op_info_ptr).operand_source_0 = data.src_0;
		(*op_info_ptr).operand_destination = data.dest;

		(*op_info_ptr).operand_source_0_num = data.aam ?
				 ((op_info_ptr->operand_source_0 == GRF_A)?
			     amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				 :data.src_0_num;

		(*op_info_ptr).operand_destination_num = data.aam ?
				((op_info_ptr->operand_destination == GRF_A)?
				amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				:data.dest_num;
	}else if(check.opcode == FILL){
		data.data_instrcution = op_info_ptr->instruction_reg;
		amm_addr.aam_addr_bit = op_info_ptr->bank_addr_reg;

		// source and destination
		(*op_info_ptr).operand_source_0 = data.src_0;
		(*op_info_ptr).operand_destination = data.dest;

		(*op_info_ptr).operand_source_0_num = data.aam ?
				 ((op_info_ptr->operand_source_0 == GRF_A)?
				 amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				 :data.src_0_num;

		(*op_info_ptr).operand_destination_num = data.aam ?
				((op_info_ptr->operand_destination == GRF_A)?
				amm_addr.grf_a_idx : amm_addr.grf_b_idx)
				:data.dest_num;

		if(fill_count[channel_idx] == 0){
			fill_count[channel_idx] = FILL_NUM;
		}else{
			program_count[channel_idx] = program_count[channel_idx] - 1;
		}
	}else{
		alu.alu_instruction = op_info_ptr->instruction_reg;

		// source and destination
		(*op_info_ptr).operand_source_0 = alu.src_0;
		(*op_info_ptr).operand_source_1 = alu.src_1;
		(*op_info_ptr).operand_destination = alu.dest;

		// srf source num
		(*op_info_ptr).srf_source_num = alu.src_2_num;

		// operand source num
		(*op_info_ptr).operand_source_0_num = alu.aam ?
				((op_info_ptr->operand_source_0 == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.src_0_num;

		(*op_info_ptr).operand_source_1_num = alu.aam ?
				((op_info_ptr->operand_source_1 == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.src_1_num;

		(*op_info_ptr).operand_destination_num = alu.aam ?
				((op_info_ptr->operand_destination == GRF_A)?
				amm_addr.grf_a_idx:amm_addr.grf_b_idx)
				:alu.dest_num;

		if(alu.aam){
			if(aam_count[channel_idx] == 0){
				aam_count[channel_idx] = AAM_NUM;
			}else{
				program_count[channel_idx] = program_count[channel_idx] - 1;
			}
		}
	}
}

void computeEngine(operandInfo *op_info_ptr, operandSource *op_source_ptr, uint32_t addr,
		uint32_t channel_idx, uint32_t *channel, hls::stream<uint32_t> & strm, uint16_t dim){
//#pragma HLS INLINE off
	switch(op_info_ptr->all_bank_pim_opcode){
		// control
		case NOP:{
			// do nothing
			break;
		}
		// change bank mode to all bank mode
		case EXIT:{
			// change bank mode all bank -> all bank pim mode
			exit_reg[channel_idx] = ENABLE_REGISTER;
			break;
		}
		// arithmetic
		// a = b+c
		case ADD:{
			addUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,
					(*op_source_ptr).execution_result);

//			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
//					op_source_ptr->execution_result, channel_idx, addr, strm, channel);
			break;
		}
		// a = b*c
		case MUL:{
			mulUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,
					(*op_source_ptr).execution_result, dim);

//			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
//					op_source_ptr->execution_result, channel_idx, addr, strm, channel);
			break;
		}
		// a += b*c+d
		case MAD:{

			madUnit(op_source_ptr->operand_0, op_source_ptr->operand_1, (*op_source_ptr).execution_result);

//			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
//					op_source_ptr->execution_result, channel_idx, addr, strm, channel);
			break;
		}
		// a += b*c
		case MAC:{
			macUnit(op_source_ptr->operand_0, op_source_ptr->operand_1,(*op_source_ptr).execution_result,
					op_source_ptr->destination_source);

//			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
//					op_source_ptr->execution_result, channel_idx, addr, strm, channel);
			break;
		}
		// move data
		case (MOV) | (FILL):{
			// write data to destination
//			writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
//					op_source_ptr->operand_0, channel_idx, addr, strm, channel);
			break;
		}
	}
}

void writeBackEngine(operandInfo *op_info_ptr, operandSource *op_source_ptr, uint32_t addr,
		uint32_t channel_idx, uint32_t *channel, hls::stream<uint32_t> & strm){

	if(op_info_ptr->all_bank_pim_opcode == ADD ||
			op_info_ptr->all_bank_pim_opcode == MUL ||
			op_info_ptr->all_bank_pim_opcode == MAC ||
			op_info_ptr->all_bank_pim_opcode == MAD){
		writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
				op_source_ptr->execution_result, channel_idx, addr, strm, channel);
	}else if(op_info_ptr->all_bank_pim_opcode == MOV ||
			op_info_ptr->all_bank_pim_opcode == FILL){
		writeResult(op_info_ptr->operand_destination, op_info_ptr->operand_destination_num,
							op_source_ptr->operand_0, channel_idx, addr, strm, channel);
	}
}

void dataWriteToReg(ap_uint<32> reg_info, uint32_t *data_from_host, uint8_t col, uint32_t channel_idx){
	if(reg_info == GRF_A_B_LOCAL_ADDR){ // grf
		writeDataToGrf(col, data_from_host, channel_idx);
	}else if(reg_info == SRF_A_M_LOCAL_ADDR){ // srf
		writeDataToSfr(data_from_host, channel_idx);
	}else if(reg_info == CRF_LOCAL_ADDR){ // crf
		writeDataToCrf(col, data_from_host, channel_idx);
	}
}

//void pimExecutionEngine(uint32_t *channel, hls::stream<uint32_t> & strm){
//	uint32_t check_opcode;
//	opcodeSplit os;
//	uint32_t check_count_clk;
//
//	operandInfo op_info;
//	operandInfo *op_info_ptr;
//	op_info_ptr = &op_info;
//
//	operandSource op_source;
//	operandSource *op_source_ptr;
//	op_source_ptr = &op_source;
//
//	jumpInfo jump_info;
//	jumpInfo *jump_info_ptr;
//	jump_info_ptr = &jump_info;
//
//	ap_uint<32> result;
//
//	static ap_uint<6> program_count;
//	ap_uint<6> jump_program_count;
//	ap_uint<32> jump_counter;
//	bank_addr_count = 0;
//
//	// pim execution start
////	for(program_count = 0; program_count < 8;){
//	//	#pragma HLS PIPELINE II=1
////#pragma HLS LOOP_FLATTEN
//
//	if(jump_info_ptr->jump_flag){
//		decodingEngine(op_info_ptr, jump_info_ptr, &jump_program_count, &jump_counter, &program_count);
//		loadEngine(op_info_ptr, op_source_ptr, strm, channel);
//		computeEngine(op_info_ptr, op_source_ptr);
//
//		if(jump_program_count == (jump_info_ptr->eob + 1)){
//			jump_program_count = jump_info_ptr->sob;
//			jump_counter = jump_counter - 1;
//		}
//
//		if(jump_counter == 0){
//			(*jump_info_ptr).jump_flag = FLAG_OFF;
//		}
//	}else{
//		decodingEngine(op_info_ptr, jump_info_ptr, &program_count, &jump_counter, &jump_program_count);
//		loadEngine(op_info_ptr, op_source_ptr, strm, channel);
//		computeEngine(op_info_ptr, op_source_ptr);
//	}
//
//	// exit
//	if(exit_reg == ENABLE_REGISTER){
//		exit_reg = 0;
//		pim_op_mode = DISABLE_REGISTER;
//	}
////	}
//}

void unpackingUnit(uint32_t *packed_data, int meta_idx, int meta_offset){
	quadToSingle q2s;
	quadToDouble q2d;
	uint16_t tmp_buf_1;
	uint16_t tmp_buf_2;

	// n_iter and n_cmd_group
	for(int i = 0; i < MAX_CHANNEL; i++){
#pragma HLS pipeline
		q2d.split_byte_double = packed_data[0 + meta_offset];
		tmp_buf_1 = q2d.first;
		n_iter[i] = tmp_buf_1;
		tmp_buf_2 = q2d.second;
		n_cmd_group[i] = tmp_buf_2;
	}

//	n_iter = (uint16_t)packed_data[0];
//	n_cmdgroup = (uint16_t)packed_data[0];

	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 16; i++){
#pragma HLS pipeline
			operand_addr_reg[j][i] = packed_data[i+1 + meta_offset];
			data_addr_reg[j][i] = packed_data[i+1+16 + meta_offset];
		}
	}

	// op code tg info
	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 4; i++){
#pragma HLS pipeline
			q2s.split_byte_single = packed_data[17+i+16 + meta_offset];
			op_code_tg[j][i*4] = q2s.first;
			op_code_tg[j][i*4+1] = q2s.second;
			op_code_tg[j][i*4+2] = q2s.third;
			op_code_tg[j][i*4+3] = q2s.forth;

			q2s.split_byte_single = packed_data[21+i+16 + meta_offset];
			addr_tg[j][i*4] = q2s.first;
			addr_tg[j][i*4+1] = q2s.second;
			addr_tg[j][i*4+2] = q2s.third;
			addr_tg[j][i*4+3] = q2s.forth;

			q2s.split_byte_single = packed_data[25+i+16 + meta_offset];
			data_tg[j][i*4] = q2s.first;
			data_tg[j][i*4+1] = q2s.second;
			data_tg[j][i*4+2] = q2s.third;
			data_tg[j][i*4+3] = q2s.forth;
		}
	}

	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 8; i++){
#pragma HLS pipeline
			q2d.split_byte_double = packed_data[29+i+16 + meta_offset];
			addr_tg_step[j][i*2] = q2d.first;
			addr_tg_step[j][i*2+1] = q2d.second;

			q2d.split_byte_double = packed_data[37+i+16 + meta_offset];
			data_tg_step[j][i*2] = q2d.first;
			data_tg_step[j][i*2+1] = q2d.second;

			q2d.split_byte_double = packed_data[45+i+16 + meta_offset];
			n_cmd[j][i*2] = q2d.first;
			n_cmd[j][i*2+1] = q2d.second;
		}
	}

	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 32; i++){
#pragma HLS pipeline
			crf_reg[j][i] = packed_data[69+i + meta_offset];
		}
	}

	last_data_size = packed_data[101 + meta_offset];

	// init data copy to buffer
	if(meta_idx == 0){
		for(int j = 0; j < 256; j++){
			for(int i = 0; i < 256; i++){
#pragma HLS pipeline
				command_gen_data[j][i] = packed_data[i+j+105];
			}
		}
	}
}

void addrGenerator(uint32_t *addr, uint32_t addr_offset,
					uint16_t addr_tg_step, uint16_t data_tg_step,
					uint8_t addr_tg, uint8_t data_tg, uint32_t channel_idx){

	uint32_t data[8];
	addrBit addr_b;

	if(addr_tg < 16){
		addr[channel_idx] = (operand_addr_reg[channel_idx][addr_tg] + addr_offset);
		for(int i = 0; i < 8; i++){
#pragma HLS unroll
			data[i] = command_gen_data[data_addr_reg[channel_idx][data_tg]][i];
		}

		if(addr_tg_step != 0){
			operand_addr_reg[channel_idx][addr_tg] = operand_addr_reg[channel_idx][addr_tg] + addr_tg_step;
		}

		if(data_tg_step != 0){
			data_addr_reg[channel_idx][data_tg] = data_addr_reg[channel_idx][data_tg] + data_tg_step;
		}
	}else{
		addr[channel_idx] = (pim_reg[channel_idx][addr_tg] + addr_offset);
		for(int i = 0; i < 8; i++){
#pragma HLS unroll
			data[i] = command_gen_data[data_addr_reg[channel_idx][data_tg]][i];
		}

		if(addr_tg_step != 0){
			pim_reg[channel_idx][addr_tg] = pim_reg[channel_idx][addr_tg] + addr_tg_step;
		}

		if(data_tg_step != 0){
			data_addr_reg[channel_idx][data_tg] = data_addr_reg[channel_idx][data_tg] + data_tg_step;
		}
	}

	addr_b.addr = *addr;
	toggleBit tb;

	if(addr_b.row == PIM_OP_MODE_LOCAL_ADDR){
		pim_op_mode[channel_idx] = ENABLE_REGISTER;
		is_store[channel_idx] = TRUE;
	}else{
		if((addr_b.row == GRF_A_B_LOCAL_ADDR) ||
				(addr_b.row == SRF_A_M_LOCAL_ADDR) ||
				(addr_b.row == BANK_ADDR_REG_ADDR) ||
				(addr_b.row == CRF_LOCAL_ADDR)){
			dataWriteToReg(addr_b.row, data, addr_b.col, channel_idx);
			is_store[channel_idx] = TRUE;
		}else if((addr_b.row == SBMR_LOCAL_ADDR) ||
				(addr_b.row == ABMR_LOCAL_ADDR)){
			pim_op_mode[channel_idx] = DISABLE_REGISTER;
			is_store[channel_idx] = TRUE;
		}
	}

	if((tb.toggle == TRUE) && (is_store[channel_idx] == FALSE)){
		// srf write
		dataWriteToReg(SRF_A_M_LOCAL_ADDR, data, addr_b.col, channel_idx);
	}
}

void cmdGenerator(uint32_t *channel, hls::stream<uint32_t> & strm, uint32_t addr_offset,
		uint32_t channel_idx, uint16_t dim, operandInfo *op_info_ptr,
		jumpInfo *jump_info_ptr, operandSource *op_source_ptr, uint32_t *addr,
		ap_uint<6> *program_count, ap_uint<6> *jump_program_count, ap_uint<32> *jump_counter){

	// first : decoding meta data
	// second : make command using meta data
	// third : fill buffer using command
	// forth : execution start
	// fifth : until execution end

	exit_reg[channel_idx] = 0;
//	for(int i = 0; i < MAX_CHANNEL; i++){
//		fill_count[i] = FILL_NUM;
//		aam_count[i] = FILL_NUM;
//	}
//
//	if(n_iter[channel_idx] == 0){
//		n_iter[channel_idx] = 1;
//	}
//
//	if(n_cmd_group[channel_idx] == 0){
//		n_cmd_group[channel_idx] = 1;
//	}
//
//	for(int i = 0; i < n_cmd_group[channel_idx]; i++){
//		if(n_cmd[channel_idx][i] == 0){
//			n_cmd[channel_idx][i] = 1;
//		}
//	}

	// pim op register ? **********
	// make command from meta data - 1 iter 1 cmd
	for(int k = 0; k < n_iter[channel_idx]; k++){
		for(int j = 0; j < n_cmd_group[channel_idx]; j++){
			for(int i = 0; i < n_cmd[channel_idx][j]; i++){
				for(int b = 0; b < BANK_NUM_PER_CHANNEL; b++){ // multi bank
#pragma HLS PIPELINE
//#pragma HLS INLINE

					addrGenerator(addr, addr_offset, addr_tg_step[channel_idx][j],
							data_tg_step[channel_idx][j], addr_tg[channel_idx][j],
							data_tg[channel_idx][j], channel_idx);

					if(is_store == FALSE){
						if(jump_info_ptr->jump_flag){
							decodingEngine(op_info_ptr, jump_info_ptr, addr[channel_idx],
									jump_program_count, jump_counter, program_count,
									channel_idx);

							loadEngine(op_info_ptr, op_source_ptr, addr[channel_idx], strm, channel,
									channel_idx);

							computeEngine(op_info_ptr, op_source_ptr, addr[channel_idx], channel_idx,
									channel, strm, dim);
							writeBackEngine(op_info_ptr, op_source_ptr, addr[channel_idx], channel_idx,
									channel, strm);

							if(jump_program_count[channel_idx] == (jump_info_ptr->eob + 1)){
								jump_program_count[channel_idx] = jump_info_ptr->sob;
								jump_counter[channel_idx] = jump_counter[channel_idx] - 1;
							}

							if(jump_counter[channel_idx] == 0){
								(*jump_info_ptr).jump_flag = FLAG_OFF;
							}
						}else{
							decodingEngine(op_info_ptr, jump_info_ptr, addr[channel_idx],
									program_count, jump_counter, jump_program_count,
									channel_idx);

							loadEngine(op_info_ptr, op_source_ptr, addr[channel_idx], strm, channel,
									channel_idx);

							computeEngine(op_info_ptr, op_source_ptr, addr[channel_idx], channel_idx,
									channel, strm, dim);
							writeBackEngine(op_info_ptr, op_source_ptr, addr[channel_idx], channel_idx,
									channel, strm);
						}

						// exit
						if(exit_reg[channel_idx] == ENABLE_REGISTER){
							exit_reg[channel_idx] = 0;
							pim_op_mode[channel_idx] = DISABLE_REGISTER;
						}
					}else{
						is_store[channel_idx] = FALSE;
					}
				}
			}
		}
	}
	//return 10;
}

void cmdInit(uint32_t init_data[100]){
	quadToSingle q2s;

	// pim reg data decoding
	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 16; i++){
			pim_reg[j][i] = init_data[i];
		}
	}

	// pim op reg data decoding
	for(int j = 0; j < MAX_CHANNEL; j++){
		for(int i = 0; i < 4; i++){
			q2s.split_byte_single = init_data[i+16];
			pim_op_reg[j][i*4] = q2s.first;
			pim_op_reg[j][i*4+1] = q2s.second;
			pim_op_reg[j][i*4+2] = q2s.third;
			pim_op_reg[j][i*4+3] = q2s.forth;
		}
	}
}

unsigned int pim(
		uint32_t *channel_0,
		uint32_t *channel_1,
		uint32_t *channel_2,
		uint32_t *channel_3,
		uint32_t *channel_4,
		uint32_t *channel_5,
		uint32_t *channel_6,
		uint32_t *channel_7,
		uint32_t *channel_8,
		uint32_t *channel_9,
		uint32_t *channel_10,
		uint32_t *channel_11,
		uint32_t *channel_12,
		uint32_t *channel_13,
		uint32_t *channel_14,
//		uint32_t *channel_15,
		uint32_t packed_data[100000]
		){
#pragma HLS INTERFACE m_axi		port=channel_0	bundle=hbm_0	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_1	bundle=hbm_1	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_2	bundle=hbm_2	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_3	bundle=hbm_3	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_4	bundle=hbm_4	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_5	bundle=hbm_5	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_6	bundle=hbm_6	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_7	bundle=hbm_7	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_8	bundle=hbm_8	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_9	bundle=hbm_9	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_10	bundle=hbm_10	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_11	bundle=hbm_11	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_12	bundle=hbm_12	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_13	bundle=hbm_13	offset=slave
#pragma HLS INTERFACE m_axi		port=channel_14	bundle=hbm_14	offset=slave
//#pragma HLS INTERFACE m_axi		port=channel_15	bundle=hbm_15	offset=slave
#pragma HLS INTERFACE s_axilite	port=packed_data			bundle=ctrl
#pragma HLS INTERFACE s_axilite	port=return	bundle=ctrl

	// read stream
	hls::stream<uint32_t> strm_0;
	hls::stream<uint32_t> strm_1;
	hls::stream<uint32_t> strm_2;
	hls::stream<uint32_t> strm_3;
	hls::stream<uint32_t> strm_4;
	hls::stream<uint32_t> strm_5;
	hls::stream<uint32_t> strm_6;
	hls::stream<uint32_t> strm_7;
	hls::stream<uint32_t> strm_8;
	hls::stream<uint32_t> strm_9;
	hls::stream<uint32_t> strm_10;
	hls::stream<uint32_t> strm_11;
	hls::stream<uint32_t> strm_12;
	hls::stream<uint32_t> strm_13;
	hls::stream<uint32_t> strm_14;
//	hls::stream<uint32_t> strm_15;

#pragma HLS stream variable=strm_0	depth=512
#pragma HLS stream variable=strm_1	depth=512
#pragma HLS stream variable=strm_2	depth=512
#pragma HLS stream variable=strm_3	depth=512
#pragma HLS stream variable=strm_4	depth=512
#pragma HLS stream variable=strm_5	depth=512
#pragma HLS stream variable=strm_6	depth=512
#pragma HLS stream variable=strm_7	depth=512
#pragma HLS stream variable=strm_8	depth=512
#pragma HLS stream variable=strm_9	depth=512
#pragma HLS stream variable=strm_10	depth=512
#pragma HLS stream variable=strm_11	depth=512
#pragma HLS stream variable=strm_12	depth=512
#pragma HLS stream variable=strm_13	depth=512
#pragma HLS stream variable=strm_14	depth=512
//#pragma HLS stream variable=strm_15	depth=512

#pragma HLS ARRAY_PARTITION	variable=n_operand_addr	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=data_addr	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=op_code_tg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=addr_tg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=data_tg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=addr_tg_step	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=data_tg_step	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=n_cmd	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=pim_op_reg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=pim_reg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=operand_addr_reg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=data_addr_reg	complete dim=2
#pragma HLS ARRAY_PARTITION	variable=crf_reg	complete dim=2

	operandInfo op_info_0;
	operandInfo *op_info_ptr_0;
	op_info_ptr_0 = &op_info_0;

	operandSource op_source_0;
	operandSource *op_source_ptr_0;
	op_source_ptr_0 = &op_source_0;

	jumpInfo jump_info_0;
	jumpInfo *jump_info_ptr_0;
	jump_info_ptr_0 = &jump_info_0;

	operandInfo op_info_1;
	operandInfo *op_info_ptr_1;
	op_info_ptr_1 = &op_info_1;

	operandSource op_source_1;
	operandSource *op_source_ptr_1;
	op_source_ptr_1 = &op_source_1;

	jumpInfo jump_info_1;
	jumpInfo *jump_info_ptr_1;
	jump_info_ptr_1 = &jump_info_1;

	operandInfo op_info_2;
	operandInfo *op_info_ptr_2;
	op_info_ptr_2 = &op_info_2;

	operandSource op_source_2;
	operandSource *op_source_ptr_2;
	op_source_ptr_2 = &op_source_2;

	jumpInfo jump_info_2;
	jumpInfo *jump_info_ptr_2;
	jump_info_ptr_2 = &jump_info_2;

	operandInfo op_info_3;
	operandInfo *op_info_ptr_3;
	op_info_ptr_3 = &op_info_3;

	operandSource op_source_3;
	operandSource *op_source_ptr_3;
	op_source_ptr_3 = &op_source_3;

	jumpInfo jump_info_3;
	jumpInfo *jump_info_ptr_3;
	jump_info_ptr_3 = &jump_info_3;

	operandInfo op_info_4;
	operandInfo *op_info_ptr_4;
	op_info_ptr_4 = &op_info_4;

	operandSource op_source_4;
	operandSource *op_source_ptr_4;
	op_source_ptr_4 = &op_source_4;

	jumpInfo jump_info_4;
	jumpInfo *jump_info_ptr_4;
	jump_info_ptr_4 = &jump_info_4;

	operandInfo op_info_5;
	operandInfo *op_info_ptr_5;
	op_info_ptr_5 = &op_info_5;

	operandSource op_source_5;
	operandSource *op_source_ptr_5;
	op_source_ptr_5 = &op_source_5;

	jumpInfo jump_info_5;
	jumpInfo *jump_info_ptr_5;
	jump_info_ptr_5 = &jump_info_5;

	operandInfo op_info_6;
	operandInfo *op_info_ptr_6;
	op_info_ptr_6 = &op_info_6;

	operandSource op_source_6;
	operandSource *op_source_ptr_6;
	op_source_ptr_6 = &op_source_6;

	jumpInfo jump_info_6;
	jumpInfo *jump_info_ptr_6;
	jump_info_ptr_6 = &jump_info_6;

	operandInfo op_info_7;
	operandInfo *op_info_ptr_7;
	op_info_ptr_7 = &op_info_7;

	operandSource op_source_7;
	operandSource *op_source_ptr_7;
	op_source_ptr_7 = &op_source_7;

	jumpInfo jump_info_7;
	jumpInfo *jump_info_ptr_7;
	jump_info_ptr_7 = &jump_info_7;

	operandInfo op_info_8;
	operandInfo *op_info_ptr_8;
	op_info_ptr_8 = &op_info_8;

	operandSource op_source_8;
	operandSource *op_source_ptr_8;
	op_source_ptr_8 = &op_source_8;

	jumpInfo jump_info_8;
	jumpInfo *jump_info_ptr_8;
	jump_info_ptr_8 = &jump_info_8;

	operandInfo op_info_9;
	operandInfo *op_info_ptr_9;
	op_info_ptr_9 = &op_info_9;

	operandSource op_source_9;
	operandSource *op_source_ptr_9;
	op_source_ptr_9 = &op_source_9;

	jumpInfo jump_info_9;
	jumpInfo *jump_info_ptr_9;
	jump_info_ptr_9 = &jump_info_9;

	operandInfo op_info_10;
	operandInfo *op_info_ptr_10;
	op_info_ptr_10 = &op_info_10;

	operandSource op_source_10;
	operandSource *op_source_ptr_10;
	op_source_ptr_10 = &op_source_10;

	jumpInfo jump_info_10;
	jumpInfo *jump_info_ptr_10;
	jump_info_ptr_10 = &jump_info_10;

	operandInfo op_info_11;
	operandInfo *op_info_ptr_11;
	op_info_ptr_11 = &op_info_11;

	operandSource op_source_11;
	operandSource *op_source_ptr_11;
	op_source_ptr_11 = &op_source_11;

	jumpInfo jump_info_11;
	jumpInfo *jump_info_ptr_11;
	jump_info_ptr_11 = &jump_info_11;

	operandInfo op_info_12;
	operandInfo *op_info_ptr_12;
	op_info_ptr_12 = &op_info_12;

	operandSource op_source_12;
	operandSource *op_source_ptr_12;
	op_source_ptr_12 = &op_source_12;

	jumpInfo jump_info_12;
	jumpInfo *jump_info_ptr_12;
	jump_info_ptr_12 = &jump_info_12;

	operandInfo op_info_13;
	operandInfo *op_info_ptr_13;
	op_info_ptr_13 = &op_info_13;

	operandSource op_source_13;
	operandSource *op_source_ptr_13;
	op_source_ptr_13 = &op_source_13;

	jumpInfo jump_info_13;
	jumpInfo *jump_info_ptr_13;
	jump_info_ptr_13 = &jump_info_13;

	operandInfo op_info_14;
	operandInfo *op_info_ptr_14;
	op_info_ptr_14 = &op_info_14;

	operandSource op_source_14;
	operandSource *op_source_ptr_14;
	op_source_ptr_14 = &op_source_14;

	jumpInfo jump_info_14;
	jumpInfo *jump_info_ptr_14;
	jump_info_ptr_14 = &jump_info_14;

	operandInfo op_info_15;
	operandInfo *op_info_ptr_15;
	op_info_ptr_15 = &op_info_15;

	operandSource op_source_15;
	operandSource *op_source_ptr_15;
	op_source_ptr_15 = &op_source_15;

	jumpInfo jump_info_15;
	jumpInfo *jump_info_ptr_15;
	jump_info_ptr_15 = &jump_info_15;

	uint32_t addr[MAX_CHANNEL];

	ap_uint<6> program_count[MAX_CHANNEL];
	ap_uint<6> jump_program_count[MAX_CHANNEL];
	ap_uint<32> jump_counter[MAX_CHANNEL];

	token_dim = 512;

	// init buffer
	// packed_data[102] == 0 : init, packed_data[99] == 1 : execute

//	if(packed_data[104] == 1){
//		unpackingUnit(packed_data);

		// cmdGenerator(channel_0, strm_0, 0, 0) -> channel, strm, addr offset, channel index
//	for(int i = 0; i < 15; i++){
////#pragma HLS unroll factor=1
//		switch(i){
//			case 0:{
//				cmdGenerator(channel_0, strm_0, 0, 0,
//						op_info_ptr_0, jump_info_ptr_0, op_source_ptr_0, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 1:{
//				cmdGenerator(channel_1, strm_1, 1, 1,
//						op_info_ptr_1, jump_info_ptr_1, op_source_ptr_1, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 2:{
//				cmdGenerator(channel_2, strm_2, 2, 2,
//						op_info_ptr_2, jump_info_ptr_2, op_source_ptr_2, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 3:{
//				cmdGenerator(channel_3, strm_3, 3, 3,
//						op_info_ptr_3, jump_info_ptr_3, op_source_ptr_3, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 4:{
//				cmdGenerator(channel_4, strm_4, 4, 4,
//						op_info_ptr_4, jump_info_ptr_4, op_source_ptr_4, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 5:{
//				cmdGenerator(channel_5, strm_5, 5, 5,
//						op_info_ptr_5, jump_info_ptr_5, op_source_ptr_5, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 6:{
//				cmdGenerator(channel_6, strm_6, 6, 6,
//						op_info_ptr_6, jump_info_ptr_6, op_source_ptr_6, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 7:{
//				cmdGenerator(channel_7, strm_7, 7, 7,
//						op_info_ptr_7, jump_info_ptr_7, op_source_ptr_7, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 8:{
//				cmdGenerator(channel_8, strm_8, 8, 8,
//						op_info_ptr_8, jump_info_ptr_8, op_source_ptr_8, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 9:{
//				cmdGenerator(channel_9, strm_9, 9, 9,
//						op_info_ptr_9, jump_info_ptr_9, op_source_ptr_9, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 10:{
//				cmdGenerator(channel_10, strm_10, 10, 10,
//						op_info_ptr_10, jump_info_ptr_10, op_source_ptr_10, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 11:{
//				cmdGenerator(channel_11, strm_11, 11, 11,
//						op_info_ptr_11, jump_info_ptr_11, op_source_ptr_11, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 12:{
//				cmdGenerator(channel_12, strm_12, 12, 12,
//						op_info_ptr_12, jump_info_ptr_12, op_source_ptr_12, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 13:{
//				cmdGenerator(channel_13, strm_13, 13, 13,
//						op_info_ptr_13, jump_info_ptr_13, op_source_ptr_13, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
//			case 14:{
//				cmdGenerator(channel_14, strm_14, 14, 14,
//						op_info_ptr_14, jump_info_ptr_14, op_source_ptr_14, addr,
//						program_count, jump_program_count, jump_counter);
//				break;
//			}
////				case 15:{
////					cmdGenerator(channel_15, strm_15, 15, 15,
////							op_info_ptr_15, jump_info_ptr_15, op_source_ptr_15, addr,
////							program_count, jump_program_count, jump_counter);
////					break;
////				}
//		}
//	}

//				case 15:{
//					cmdGenerator(channel_15, strm_15, 15, 15,
//							op_info_ptr_15, jump_info_ptr_15, op_source_ptr_15, addr,
//							program_count, jump_program_count, jump_counter);
//	}else{
//		cmdInit(packed_data);
//	}

	if(packed_data[104] == 1){
		for(int i = 0; i < packed_data[103]; i++){
			switch(i){
				case 0:{
					unpackingUnit(packed_data, i, FIRST_META_DATA_OFFSET);
					break;
				}
				case 1:{
					unpackingUnit(packed_data, i, SECOND_META_DATA_OFFSET);
					break;
				}
				case 2:{
					unpackingUnit(packed_data, i, THIRD_META_DATA_OFFSET);
					break;
				}
				case 3:{
					unpackingUnit(packed_data, i, FORTH_META_DATA_OFFSET);
					break;
				}
				case 4:{
					unpackingUnit(packed_data, i, FIFTH_META_DATA_OFFSET);
					break;
				}
				case 5:{
					unpackingUnit(packed_data, i, SIXTH_META_DATA_OFFSET);
					break;
				}
			}

			cmdGenerator(channel_0, strm_0, 0, 0, token_dim,
					op_info_ptr_0, jump_info_ptr_0, op_source_ptr_0, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_1, strm_1, 1, 1, token_dim,
					op_info_ptr_1, jump_info_ptr_1, op_source_ptr_1, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_2, strm_2, 2, 2, token_dim,
					op_info_ptr_2, jump_info_ptr_2, op_source_ptr_2, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_3, strm_3, 3, 3, token_dim,
					op_info_ptr_3, jump_info_ptr_3, op_source_ptr_3, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_4, strm_4, 4, 4, token_dim,
					op_info_ptr_4, jump_info_ptr_4, op_source_ptr_4, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_5, strm_5, 5, 5, token_dim,
					op_info_ptr_5, jump_info_ptr_5, op_source_ptr_5, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_6, strm_6, 6, 6, token_dim,
					op_info_ptr_6, jump_info_ptr_6, op_source_ptr_6, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_7, strm_7, 7, 7, token_dim,
					op_info_ptr_7, jump_info_ptr_7, op_source_ptr_7, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_8, strm_8, 8, 8, token_dim,
					op_info_ptr_8, jump_info_ptr_8, op_source_ptr_8, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_9, strm_9, 9, 9, token_dim,
					op_info_ptr_9, jump_info_ptr_9, op_source_ptr_9, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_10, strm_10, 10, 10, token_dim,
					op_info_ptr_10, jump_info_ptr_10, op_source_ptr_10, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_11, strm_11, 11, 11, token_dim,
					op_info_ptr_11, jump_info_ptr_11, op_source_ptr_11, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_12, strm_12, 12, 12, token_dim,
					op_info_ptr_12, jump_info_ptr_12, op_source_ptr_12, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_13, strm_13, 13, 13, token_dim,
					op_info_ptr_13, jump_info_ptr_13, op_source_ptr_13, addr,
					program_count, jump_program_count, jump_counter);
			cmdGenerator(channel_14, strm_14, 14, 14, token_dim,
					op_info_ptr_14, jump_info_ptr_14, op_source_ptr_14, addr,
					program_count, jump_program_count, jump_counter);
		}
	}else{
			cmdInit(packed_data);
	}

	return 1;
}

