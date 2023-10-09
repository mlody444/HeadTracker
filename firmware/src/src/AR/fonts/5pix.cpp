#include <stdio.h>

static const uint16_t pix5_empty[] = {
	0x000F, 0x0009, 0x0009, 0x0009, 0x000F
};


static const uint16_t pix5_0x20[] = {	// '   '
	0x0000, 0x0000, 0x0000, 0x0000
};

static const uint16_t pix5_0x21[] = {	// ' ! '
	0x0000, 0x0017, 0x0000, 0x0000
};

static const uint16_t pix5_0x22[] = {	// ' " '
	0x0003, 0x0000, 0x0003, 0x0000
};

static const uint16_t pix5_0x23[] = {	// ' # '
	0x001F, 0x000A, 0x001F, 0x000A
};

static const uint16_t pix5_0x24[] = {	// ' $ '
	0x0012, 0x001D, 0x0017, 0x0009
};

static const uint16_t pix5_0x25[] = {	// ' % '
	0x0009, 0x0005, 0x0014, 0x0012
};

static const uint16_t pix5_0x26[] = {	// ' & '
	0x000A, 0x0015, 0x000A, 0x0010
};

static const uint16_t pix5_0x27[] = {	// ' ' '
	0x0000, 0x0003, 0x0000, 0x0000
};

static const uint16_t pix5_0x28[] = {	// ' ( '
	0x0000, 0x000E, 0x0011, 0x0000
};

static const uint16_t pix5_0x29[] = {	// ' ) '
	0x0000, 0x0011, 0x000E, 0x0000
};

static const uint16_t pix5_0x2A[] = {	// ' * '
	0x0015, 0x000E, 0x000E, 0x0015
};

static const uint16_t pix5_0x2B[] = {	// ' + '
	0x0004, 0x000E, 0x0004, 0x0000
};

static const uint16_t pix5_0x2C[] = {	// ' , '
	0x0010, 0x0008, 0x0000, 0x0000
};

static const uint16_t pix5_0x2D[] = {	// ' - '
	0x0004, 0x0004, 0x0004, 0x0000
};

static const uint16_t pix5_0x2E[] = {	// ' . '
	0x0000, 0x0010, 0x0000, 0x0000
};

static const uint16_t pix5_0x2F[] = {	// ' / '
	0x0018, 0x0004, 0x0003, 0x0000
};

static const uint16_t pix5_0x30[] = {	// ' 0 '
	0x000E, 0x0015, 0x0013, 0x000E
};

static const uint16_t pix5_0x31[] = {	// ' 1 '
	0x0000, 0x0002, 0x001F, 0x0000
};

static const uint16_t pix5_0x32[] = {	// ' 2 '
	0x0012, 0x0019, 0x0015, 0x0012
};

static const uint16_t pix5_0x33[] = {	// ' 3 '
	0x0011, 0x0015, 0x0015, 0x000A
};

static const uint16_t pix5_0x34[] = {	// ' 4 '
	0x0007, 0x0004, 0x0004, 0x001F
};

static const uint16_t pix5_0x35[] = {	// ' 5 '
	0x0017, 0x0015, 0x0015, 0x0009
};

static const uint16_t pix5_0x36[] = {	// ' 6 '
	0x000E, 0x0015, 0x0015, 0x0008
};

static const uint16_t pix5_0x37[] = {	// ' 7 '
	0x0001, 0x0001, 0x001D, 0x0003
};

static const uint16_t pix5_0x38[] = {	// ' 8 '
	0x000A, 0x0015, 0x0015, 0x000A
};

static const uint16_t pix5_0x39[] = {	// ' 9 '
	0x0002, 0x0015, 0x0015, 0x000E
};

static const uint16_t pix5_0x3A[] = {	// ' : '
	0x0000, 0x000A, 0x0000, 0x0000
};

static const uint16_t pix5_0x3B[] = {	// ' ; '
	0x0000, 0x001A, 0x0000, 0x0000
};

static const uint16_t pix5_0x3C[] = {	// ' < '
	0x0004, 0x000A, 0x0011, 0x0000
};

static const uint16_t pix5_0x3D[] = {	// ' = '
	0x000A, 0x000A, 0x000A, 0x0000
};

static const uint16_t pix5_0x3E[] = {	// ' > '
	0x0011, 0x000A, 0x0004, 0x0000
};

static const uint16_t pix5_0x3F[] = {	// ' ? '
	0x0001, 0x0015, 0x0002, 0x0000
};

static const uint16_t pix5_0x40[] = {	// ' @ '
	0x000E, 0x0011, 0x0015, 0x0016
};

static const uint16_t pix5_0x41[] = {	// ' A '
	0x001E, 0x0005, 0x0005, 0x001E
};

static const uint16_t pix5_0x42[] = {	// ' B '
	0x001F, 0x0015, 0x0015, 0x000A
};

static const uint16_t pix5_0x43[] = {	// ' C '
	0x000E, 0x0011, 0x0011, 0x000A
};

static const uint16_t pix5_0x44[] = {	// ' D '
	0x001F, 0x0011, 0x0011, 0x000E
};

static const uint16_t pix5_0x45[] = {	// ' E '
	0x001F, 0x0015, 0x0015, 0x0011
};

static const uint16_t pix5_0x46[] = {	// ' F '
	0x001F, 0x0005, 0x0005, 0x0001
};

static const uint16_t pix5_0x47[] = {	// ' G '
	0x000E, 0x0011, 0x0015, 0x000C
};

static const uint16_t pix5_0x48[] = {	// ' H '
	0x001F, 0x0004, 0x0004, 0x001F
};

static const uint16_t pix5_0x49[] = {	// ' I '
	0x0000, 0x001F, 0x0000, 0x0000
};

static const uint16_t pix5_0x4A[] = {	// ' J '
	0x0008, 0x0010, 0x0010, 0x000F
};

static const uint16_t pix5_0x4B[] = {	// ' K '
	0x001F, 0x0004, 0x000A, 0x0011
};

static const uint16_t pix5_0x4C[] = {	// ' L '
	0x001F, 0x0010, 0x0010, 0x0010
};

static const uint16_t pix5_0x4D[] = {	// ' M '
	0x001F, 0x0002, 0x0002, 0x001F
};

static const uint16_t pix5_0x4E[] = {	// ' N '
	0x001F, 0x0002, 0x0004, 0x001F
};

static const uint16_t pix5_0x4F[] = {	// ' O '
	0x000E, 0x0011, 0x0011, 0x000E
};

static const uint16_t pix5_0x50[] = {	// ' P '
	0x001F, 0x0005, 0x0005, 0x0002
};

static const uint16_t pix5_0x51[] = {	// ' Q '
	0x000E, 0x0011, 0x0019, 0x001E
};

static const uint16_t pix5_0x52[] = {	// ' R '
	0x001F, 0x0005, 0x000D, 0x0012
};

static const uint16_t pix5_0x53[] = {	// ' S '
	0x0012, 0x0015, 0x0015, 0x0009
};

static const uint16_t pix5_0x54[] = {	// ' T '
	0x0001, 0x001F, 0x0001, 0x0000
};

static const uint16_t pix5_0x55[] = {	// ' U '
	0x000F, 0x0010, 0x0010, 0x000F
};

static const uint16_t pix5_0x56[] = {	// ' V '
	0x000F, 0x0010, 0x000F, 0x0000
};

static const uint16_t pix5_0x57[] = {	// ' W '
	0x001F, 0x0008, 0x0008, 0x001F
};

static const uint16_t pix5_0x58[] = {	// ' X '
	0x001B, 0x0004, 0x0004, 0x001B
};

static const uint16_t pix5_0x59[] = {	// ' Y '
	0x0003, 0x001C, 0x0003, 0x0000
};

static const uint16_t pix5_0x5A[] = {	// ' Z '
	0x0019, 0x0015, 0x0015, 0x0013
};

static const uint16_t pix5_0x5B[] = {	// ' [ '
	0x0000, 0x001F, 0x0011, 0x0000
};

static const uint16_t pix5_0x5C[] = {	// ' \ '
	0x0003, 0x0004, 0x0018, 0x0000
};

static const uint16_t pix5_0x5D[] = {	// ' ] '
	0x0000, 0x0011, 0x001F, 0x0000
};

static const uint16_t pix5_0x5E[] = {	// ' ^ '
	0x0002, 0x0001, 0x0002, 0x0000
};

static const uint16_t pix5_0x5F[] = {	// ' _ '
	0x0010, 0x0010, 0x0010, 0x0010
};

static const uint16_t pix5_0x60[] = {	// ' ` '
	0x0000, 0x0001, 0x0002, 0x0000
};

static const uint16_t pix5_0x7B[] = {	// ' { '
	0x0004, 0x001F, 0x0011, 0x0000
};

static const uint16_t pix5_0x7C[] = {	// ' | '
	0x0000, 0x001B, 0x0000, 0x0000
};

static const uint16_t pix5_0x7D[] = {	// ' } '
	0x0011, 0x001F, 0x0004, 0x0000
};


const uint16_t *pix5[106] = {
	pix5_0x20,
	pix5_0x21,
	pix5_0x22,
	pix5_0x23,
	pix5_0x24,
	pix5_0x25,
	pix5_0x26,
	pix5_0x27,
	pix5_0x28,
	pix5_0x29,
	pix5_0x2A,
	pix5_0x2B,
	pix5_0x2C,
	pix5_0x2D,
	pix5_0x2E,
	pix5_0x2F,
	pix5_0x30,
	pix5_0x31,
	pix5_0x32,
	pix5_0x33,
	pix5_0x34,
	pix5_0x35,
	pix5_0x36,
	pix5_0x37,
	pix5_0x38,
	pix5_0x39,
	pix5_0x3A,
	pix5_0x3B,
	pix5_0x3C,
	pix5_0x3D,
	pix5_0x3E,
	pix5_0x3F,
	pix5_0x40,
	pix5_0x41,
	pix5_0x42,
	pix5_0x43,
	pix5_0x44,
	pix5_0x45,
	pix5_0x46,
	pix5_0x47,
	pix5_0x48,
	pix5_0x49,
	pix5_0x4A,
	pix5_0x4B,
	pix5_0x4C,
	pix5_0x4D,
	pix5_0x4E,
	pix5_0x4F,
	pix5_0x50,
	pix5_0x51,
	pix5_0x52,
	pix5_0x53,
	pix5_0x54,
	pix5_0x55,
	pix5_0x56,
	pix5_0x57,
	pix5_0x58,
	pix5_0x59,
	pix5_0x5A,
	pix5_0x5B,
	pix5_0x5C,
	pix5_0x5D,
	pix5_0x5E,
	pix5_0x5F,
	pix5_0x60,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_empty,
	pix5_0x7B,
	pix5_0x7C,
	pix5_0x7D
};
