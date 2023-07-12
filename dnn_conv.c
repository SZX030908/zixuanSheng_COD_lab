#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	unsigned conv_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	unsigned weight_general_size;
	unsigned conv_general_size;
	unsigned in_general_size;

	unsigned conv_count_offset;
	unsigned conv_x_offset;
	unsigned conv_y_offset;

	unsigned input_count_offset;
	unsigned input_x_offset;
	unsigned input_y_offset;

	unsigned weight_count_offset;
	unsigned weight_x_offset;
	unsigned weight_y_offset;

	unsigned conv_y,conv_x;
	unsigned kx,ky;
	unsigned input_x,input_y;

	short multi_input,multi_weight;

	int result;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;//输出图中与y对应的，纵坐标的尺寸//24
	conv_size.d3 = conv_out_w;//输出图中与x对应的，横坐标的尺寸//24

	weight_general_size = (unsigned)mul(weight_size.d2,weight_size.d3) + 1;
	conv_general_size = (unsigned)mul(conv_size.d2,conv_size.d3);
	in_general_size = (unsigned)mul(input_fm_w,input_fm_h);


	for(conv_offset = 0;conv_offset < conv_size.d1;conv_offset++)
	{
		conv_count_offset = (unsigned)mul(conv_offset,conv_general_size);
		weight_count_offset = (unsigned)mul(mul(conv_offset,rd_size.d1),weight_general_size);

		//输入通道的标号数坐标
		for(input_offset = 0;input_offset < conv_size.d0;input_offset++)
		{
			input_count_offset = (unsigned)mul(input_offset,in_general_size);
			weight_count_offset = weight_count_offset + (unsigned)mul(input_offset,weight_general_size);

			//输出通道的标号数坐标
			for(conv_y = 0;conv_y < conv_out_h;conv_y++)
			{
				conv_y_offset = (unsigned)mul(conv_y,conv_out_w);
				
				//确定输出通道现需要算的值的纵坐标产生的偏移
				for(conv_x = 0;conv_x < conv_out_w;conv_x++)
				{
					conv_x_offset = conv_x;
					//确定输出通道现需要算的值的横坐标产生的偏移
					if(!input_offset)
						result = ((int)*(weight + weight_count_offset)) << FRAC_BIT;
					else
						result = 0;
					//确定bias值

					for(ky = 0;ky < weight_size.d2;ky++)
					{
						weight_y_offset = (unsigned)mul(ky,weight_size.d3);
						//确定卷积通道现需要算的值的纵坐标产生的偏移
						for(kx = 0;kx < weight_size.d3;kx++)
						{
							weight_x_offset = kx;
							//确定卷积通道现需要算的值的横坐标产生的偏移

							input_y = ky + (unsigned)mul(conv_y,stride) - pad;
							input_x = kx + (unsigned)mul(conv_x,stride) - pad;

							input_y_offset = (unsigned)mul(input_y,input_fm_w);
							input_x_offset = input_x;
							//确定输入通道现需要算的值的纵坐标产生的偏移
							//确定输入通道现需要算的值的横坐标产生的偏移

							if(input_x >=0 && input_x < input_fm_w && input_y >= 0 && input_y < input_fm_h)
							{
								multi_input = *(in + input_count_offset + input_y_offset + input_x_offset);
								multi_weight = *(weight + weight_count_offset + weight_y_offset + weight_x_offset + 1);
								result += mul(multi_input,multi_weight);
							}
							
						}
					}
					*(out + conv_count_offset + conv_y_offset + conv_x_offset) = (short)(result >> FRAC_BIT); 
				}
			}
		}    
	}

	//TODO: Please add your implementation here
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	unsigned conv_general_size;
	unsigned output_general_size;

	unsigned conv_count_offset;
	unsigned output_count_offset;
	unsigned conv_count_offset_start;
	unsigned output_count_offset_start;
	unsigned output_y_offset;
	unsigned output_x_offset;

	unsigned x,y;//决定out_x,out_y
	unsigned ix,iy;//小方格内循环

	unsigned conv_x,conv_y;
	unsigned temp_x,temp_y;

	unsigned temp_x_offset,temp_y_offset;
	short short_max;
	short temp;

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	//conv_size.d0,1
	//conv_size.d1,20
	//conv_size.d2,24
	//conv_size.d3,24

	conv_general_size = (unsigned)mul(input_fm_h,input_fm_w);
	output_general_size = (unsigned)mul(pool_out_h,pool_out_w);

	for(output_offset = 0;output_offset < conv_size.d0;output_offset++)
	{
		conv_count_offset_start = (unsigned)mul(conv_general_size,mul(output_offset,conv_size.d1));
		output_count_offset_start = (unsigned)mul(output_general_size,mul(output_offset,conv_size.d1));
		for(input_offset = 0;input_offset < conv_size.d1;input_offset++)
		{
			output_count_offset = output_count_offset_start + (unsigned)mul(output_general_size,input_offset);
			conv_count_offset = conv_count_offset_start + (unsigned)mul(conv_general_size,input_offset);

			for(y = 0;y < pool_out_h;y++)
			{
				output_y_offset = (unsigned)mul(y,pool_out_w);
				for(x = 0;x < pool_out_w;x++)
				{
					output_x_offset = x;
					//确定已经知道输出通道的偏移量，分别循环该输出通道中x和y的值

					conv_y = (unsigned)mul(y,stride) - pad;
					conv_x = (unsigned)mul(x,stride) - pad;

					short_max = 0x8000;

					for(iy = 0;iy < stride;iy++)
					{
						for(ix = 0;ix < stride;ix++)
						{
							//开始对一个一个小方格进行循环，来求最大值
							temp_y = conv_y + iy;
							temp_x = conv_x + ix;

							temp_y_offset = (unsigned)mul(temp_y,input_fm_w);
							temp_x_offset = temp_x;

							if(temp_x >=0 && temp_x < input_fm_w && temp_y >= 0 && temp_y < input_fm_h)
							{
								temp = *(out + conv_count_offset + temp_y_offset + temp_x_offset);

								if(short_max < temp)
									short_max = temp;
							}
						}
					}

					*(out + output_count_offset + output_y_offset + output_x_offset) = short_max; 
				}
			}
		}    
	}
	//TODO: Please add your implementation here

}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	int temp_start;

	temp_start  = *gpio_start;
	temp_start |= 1;
	*gpio_start = temp_start;

	while(!(*gpio_done & 1))
		;

	return;
	//TODO: Please add your implementation here
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	res.msec = 0;

	bench_prepare(&res);
	
#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif

	bench_done(&res);
	printf("total cycle %u\n",res.msec);

	int result = comparing();
	printf("benchmark finished\n");

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}
