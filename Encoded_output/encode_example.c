#include <stdio.h>



// 0xC began transmission 

void data_encode(int16_t x, int16_t y, int16_t z, char *pkt)
{
	// x
	pkt[0] =        (x >> 12 & 0xF);
	pkt[1] = 0x10 | (x >> 8  & 0xF);
	pkt[2] = 0x20 | (x >> 4  & 0xF);
	pkt[3] = 0x30 | (x       & 0xF);
	//y
	pkt[4] = 0x40 | (y >> 12 & 0xF);
	pkt[5] = 0x50 | (y >> 8  & 0xF);
	pkt[6] = 0x60 | (y >> 4  & 0xF);
	pkt[7] = 0x70 | (y       & 0xF);
	//z
	pkt[8]  = 0x80 | (z >> 12 & 0xF);
	pkt[9]  = 0x90 | (z >> 8  & 0xF);
	pkt[10] = 0xA0 | (z >> 4  & 0xF);
	pkt[11] = 0xB0 | (z       & 0xF);
}

int16_t data_decode_x(char *pkt)
{
	// x
	int16_t x = 0;
	x = (pkt[0]<<12 & 0xF000)| (pkt[1]<<8 & 0xF00)| (pkt[2]<<4 & 0xF0)|(pkt[3] & 0xF) ;
	return x;

}


int16_t data_decode_y(char *pkt)
{
	// x
	int16_t y = 0;
	y = (pkt[4]<<12 & 0xF000)| (pkt[5]<<8 & 0xF00)| (pkt[6]<<4 & 0xF0)|(pkt[7] & 0xF) ;
	return y;

}

int16_t data_decode_z(char *pkt)
{
	// x
	int16_t z = 0;
	z = (pkt[8]<<12 & 0xF000)| (pkt[9]<<8 & 0xF00)| (pkt[10]<<4 & 0xF0)|(pkt[11] & 0xF) ;
	return z;

}






int main()
{

	int16_t x = -1234;
	int16_t y = 2;
	int16_t z = -789;

	int16_t x_rcv = 0;
	int16_t y_rcv = 0;
	int16_t z_rcv = 0;

	char *enc_datapkt;

    // encode
	enc_datapkt = (char*) malloc(12 * sizeof(char));
	data_encode(x,y,z, enc_datapkt);

	printf("%d\n", x);
    
    // decode
	x_rcv = data_decode_x(enc_datapkt);
	y_rcv = data_decode_y(enc_datapkt);
	z_rcv = data_decode_z(enc_datapkt);

	printf("x: %d\n", x_rcv);
	printf("y: %d\n", y_rcv);
	printf("z: %d\n", z_rcv);
    
	return 0;

}