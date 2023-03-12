/*
 * Small jpeg decoder library
 *
 * Copyright (c) 2006, Luc Saillard <luc@saillard.org>
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * - Neither the name of the author nor the names of its contributors may be
 *  used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "tinyjpeg.h"
#include "tinyjpeg-internal.h"

enum std_markers
{
  DQT = 0xDB,  /* Define Quantization Table */
  SOF = 0xC0,  /* Start of Frame (size information) */
  DHT = 0xC4,  /* Huffman Table */
  SOI = 0xD8,  /* Start of Image */
  SOS = 0xDA,  /* Start of Scan */
  RST = 0xD0,  /* Reset Marker d0 -> .. */
  RST7 = 0xD7, /* Reset Marker .. -> d7 */
  EOI = 0xD9,  /* End of Image */
  DRI = 0xDD,  /* Define Restart Interval */
  APP0 = 0xE0,
};

#define cY 0
#define cCb 1
#define cCr 2

#define BLACK_Y 0
#define BLACK_U 127
#define BLACK_V 127

#if DEBUG
#define trace(fmt, args...)       \
  do                              \
  {                               \
    fprintf(stderr, fmt, ##args); \
    fflush(stderr);               \
  } while (0)
#else
#define trace(fmt, args...) \
  do                        \
  {                         \
  } while (0)
#endif
#define error(fmt, args...)                                    \
  do                                                           \
  {                                                            \
    snprintf(error_string, sizeof(error_string), fmt, ##args); \
    return -1;                                                 \
  } while (0)

#if 0
static char *print_bits(unsigned int value, char *bitstr)
{
  int i, j;
  i=31;
  while (i>0)
   {
     if (value & (1UL<<i))
       break;
     i--;
   }
  j=0;
  while (i>=0)
   {
     bitstr[j++] = (value & (1UL<<i))?'1':'0';
     i--;
   }
  bitstr[j] = 0;
  return bitstr;
}

static void print_next_16bytes(int offset, const unsigned char *stream)
{
  trace("%4.4x: %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x\n",
	offset,
	stream[0], stream[1], stream[2], stream[3], 
	stream[4], stream[5], stream[6], stream[7],
	stream[8], stream[9], stream[10], stream[11], 
	stream[12], stream[13], stream[14], stream[15]);
}

#endif

/* Global variable to return the last error found while deconding */
static char error_string[256];

/**
 * @brief 曲折扫描表
 */
static const unsigned char zigzag[64] =
    {
        0, 1, 5, 6, 14, 15, 27, 28,
        2, 4, 7, 13, 16, 26, 29, 42,
        3, 8, 12, 17, 25, 30, 41, 43,
        9, 11, 18, 24, 31, 40, 44, 53,
        10, 19, 23, 32, 39, 45, 52, 54,
        20, 22, 33, 38, 46, 51, 55, 60,
        21, 34, 37, 47, 50, 56, 59, 61,
        35, 36, 48, 49, 57, 58, 62, 63};

/* Set up the standard Huffman tables (cf. JPEG standard section K.3) */
/* IMPORTANT: these are only valid for 8-bit data precision! */
static const unsigned char bits_dc_luminance[17] =
    {
        0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
static const unsigned char val_dc_luminance[] =
    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const unsigned char bits_dc_chrominance[17] =
    {
        0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0};
static const unsigned char val_dc_chrominance[] =
    {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

static const unsigned char bits_ac_luminance[17] =
    {
        0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d};
static const unsigned char val_ac_luminance[] =
    {
        0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
        0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
        0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
        0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
        0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
        0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
        0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
        0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
        0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
        0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
        0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
        0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
        0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
        0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
        0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
        0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
        0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
        0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
        0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
        0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
        0xf9, 0xfa};

static const unsigned char bits_ac_chrominance[17] =
    {
        0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77};

static const unsigned char val_ac_chrominance[] =
    {
        0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
        0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
        0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
        0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
        0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
        0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
        0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
        0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
        0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
        0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
        0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
        0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
        0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
        0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
        0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
        0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
        0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
        0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
        0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
        0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
        0xf9, 0xfa};

/*
 * 4 functions to manage the stream
 *
 *  fill_nbits: put at least nbits in the reservoir of bits.
 *              But convert any 0xff,0x00 into 0xff
 *  get_nbits: read nbits from the stream, and put it in result,
 *             bits is removed from the stream and the reservoir is filled
 *             automaticaly. The result is signed according to the number of
 *             bits.
 *  look_nbits: read nbits from the stream without marking as read.
 *  skip_nbits: read nbits from the stream but do not return the result.
 *
 * stream: current pointer in the jpeg data (read bytes per bytes)
 * nbits_in_reservoir: number of bits filled into the reservoir
 * reservoir: register that contains bits information. Only nbits_in_reservoir
 *            is valid.
 *                          nbits_in_reservoir
 *                        <--    17 bits    -->
 *            Ex: 0000 0000 1010 0000 1111 0000   <== reservoir
 *                        ^
 *                        bit 1
 *            To get two bits from this example
 *                 result = (reservoir >> 15) & 3
 *
 */
#define fill_nbits(reservoir, nbits_in_reservoir, stream, nbits_wanted) \
  do                                                                    \
  {                                                                     \
    while (nbits_in_reservoir < nbits_wanted)                           \
    {                                                                   \
      unsigned char c;                                                  \
      if (stream >= priv->stream_end)                                   \
        longjmp(priv->jump_state, -EIO);                                \
      c = *stream++;                                                    \
      reservoir <<= 8;                                                  \
      if (c == 0xff && *stream == 0x00)                                 \
        stream++;                                                       \
      reservoir |= c;                                                   \
      nbits_in_reservoir += 8;                                          \
    }                                                                   \
  } while (0);

/* Signed version !!!! */
#define get_nbits(reservoir, nbits_in_reservoir, stream, nbits_wanted, result) \
  do                                                                           \
  {                                                                            \
    fill_nbits(reservoir, nbits_in_reservoir, stream, (nbits_wanted));         \
    result = ((reservoir) >> (nbits_in_reservoir - (nbits_wanted)));           \
    nbits_in_reservoir -= (nbits_wanted);                                      \
    reservoir &= ((1U << nbits_in_reservoir) - 1);                             \
    if ((unsigned int)result < (1UL << ((nbits_wanted)-1)))                    \
      result += (0xFFFFFFFFUL << (nbits_wanted)) + 1;                          \
  } while (0);

#define look_nbits(reservoir, nbits_in_reservoir, stream, nbits_wanted, result) \
  do                                                                            \
  {                                                                             \
    fill_nbits(reservoir, nbits_in_reservoir, stream, (nbits_wanted));          \
    result = ((reservoir) >> (nbits_in_reservoir - (nbits_wanted)));            \
  } while (0);

/* To speed up the decoding, we assume that the reservoir have enough bit
 * slow version:
 * #define skip_nbits(reservoir,nbits_in_reservoir,stream,nbits_wanted) do { \
 *   fill_nbits(reservoir,nbits_in_reservoir,stream,(nbits_wanted)); \
 *   nbits_in_reservoir -= (nbits_wanted); \
 *   reservoir &= ((1U<<nbits_in_reservoir)-1); \
 * }  while(0);
 */
#define skip_nbits(reservoir, nbits_in_reservoir, stream, nbits_wanted) \
  do                                                                    \
  {                                                                     \
    nbits_in_reservoir -= (nbits_wanted);                               \
    reservoir &= ((1U << nbits_in_reservoir) - 1);                      \
  } while (0);

#define be16_to_cpu(x) (((x)[0] << 8) | (x)[1])

static void resync(struct jdec_private *priv);

/**
 * Get the next (valid) huffman code in the stream.
 *
 * To speedup the procedure, we look HUFFMAN_HASH_NBITS bits and the code is
 * lower than HUFFMAN_HASH_NBITS we have automaticaly the length of the code
 * and the value by using two lookup table.
 * Else if the value is not found, just search (linear) into an array for each
 * bits is the code is present.
 *
 * If the code is not present for any reason, -1 is return.
 */
static int get_next_huffman_code(struct jdec_private *priv, struct huffman_table *huffman_table)
{
  int value, hcode;
  unsigned int extra_nbits, nbits;
  uint16_t *slowtable;
  // priv->stream 在header sos 解析后，其指向开始decode的位置。priv->reservoir,nbits_in_reservoir 在开始decode前(resync)被初始化0
  look_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, HUFFMAN_HASH_NBITS, hcode); // 取前9位，用于快速检索
  value = huffman_table->lookup[hcode];                                                           //  快速检索
  if (__likely(value >= 0))
  {
    unsigned int code_size = huffman_table->code_size[value];
    skip_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, code_size); // 保留当前的进度，便于下次继续？
    return value;
  }

  // 当出现前9位仍然没有匹配到symbol的时候，需要继续从慢检索池内匹配（9->16)位所代表symbol的情况
  /* Decode more bits each time ... */
  for (extra_nbits = 0; extra_nbits < 16 - HUFFMAN_HASH_NBITS; extra_nbits++)
  {
    nbits = HUFFMAN_HASH_NBITS + 1 + extra_nbits; // 从9位开始，依次遍历到16位的情况，过程中有匹配到symbol，则直接跳出

    look_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, nbits, hcode);
    slowtable = huffman_table->slowtable[extra_nbits]; //
    /* Search if the code is in this array */
    while (slowtable[0])
    {
      if (slowtable[0] == hcode)
      {
        skip_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, nbits);
        return slowtable[1]; // 匹配到对应符号，直接返回symbol
      }
      slowtable += 2;
    }
  }
  return 0;
}

/**
 *
 * Decode a single block that contains the DCT coefficients.
 * The table coefficients is already dezigzaged at the end of the operation.
 *
 */
static void process_Huffman_data_unit(struct jdec_private *priv, int component)
{
  unsigned char j;
  unsigned int huff_code;
  unsigned char size_val, count_0;

  struct component *c = &priv->component_infos[component];
  short int DCT[64];

  /* Initialize the DCT coef table */
  memset(DCT, 0, sizeof(DCT));

  /* DC coefficient decoding */                         // 8*8的方格，左上角第一位是DC量
  huff_code = get_next_huffman_code(priv, c->DC_table); //  将霍夫曼code 转回 symbol
  // trace("+ %x\n", huff_code);
  if (huff_code)
  {
    get_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, huff_code, DCT[0]);
    DCT[0] += c->previous_DC; // 为了节约资源，多个不同的64方格的DC 变量会再量化一次，会需要参考到多个DC变量的相关关系
    c->previous_DC = DCT[0];  // 初始previous_DC=0
  }
  else
  {
    DCT[0] = c->previous_DC;
  }

  /* AC coefficient decoding */
  j = 1;
  while (j < 64)
  {
    huff_code = get_next_huffman_code(priv, c->AC_table);
    // trace("- %x\n", huff_code);

    size_val = huff_code & 0xF;
    count_0 = huff_code >> 4;

    if (size_val == 0)
    { /* RLE */
      if (count_0 == 0)
        break; /* EOB found, go out */
      else if (count_0 == 0xF)
        j += 16; /* skip 16 zeros */
    }
    else
    {
      j += count_0; /* skip count_0 zeroes */
      if (__unlikely(j >= 64))
      {
        snprintf(error_string, sizeof(error_string), "Bad huffman data (buffer overflow)");
        break;
      }
      get_nbits(priv->reservoir, priv->nbits_in_reservoir, priv->stream, size_val, DCT[j]);
      j++;
    }
  }

  for (j = 0; j < 64; j++)
    c->DCT[j] = DCT[zigzag[j]]; // 64像素zigzag化
}
/*
 * Takes two array of bits, and build the huffman table for size, and code
 *
 * lookup will return the symbol if the code is less or equal than HUFFMAN_HASH_NBITS.
 * code_size will be used to known how many bits this symbol is encoded.
 * slowtable will be used when the first lookup didn't give the result.
 */
/**
 * vals: 16位行标记后的树解码数据(symbol)这些数据将被霍夫曼树代替索引
 */
static void build_huffman_table(const unsigned char *bits, const unsigned char *vals, struct huffman_table *table)
{
  unsigned int i, j, code, code_size, val, nbits;
  unsigned char huffsize[HUFFMAN_BITS_SIZE + 1], *hz;
  unsigned int huffcode[HUFFMAN_BITS_SIZE + 1], *hc;
  int next_free_entry;

  /*
   * Build a temp array
   *   huffsize[X] => numbers of bits to write vals[X]
   */
  hz = huffsize;            // huffsize 这个结构是16*16的大小
  for (i = 1; i <= 16; i++) // 之后前16位代表每行中bit的数量
  {
    for (j = 1; j <= bits[i]; j++) // bits是前面解析的前16位数据，如果bits[i]为0，说明这行的数据为0，会直接跳过该行
      *hz++ = i;                   // 按照译码码长，给16个数值所代表的行和所需要的数值量大小占位eg: 11 222 3 4444 对应(1:2,2:3,3:1,4:4)
  }
  *hz = 0; // 避免空指针

  // 初始化索引缓冲
  memset(table->lookup, 0xff, sizeof(table->lookup)); //  构建快速索引缓冲,在得到霍夫曼编码树后，可以用于快速解码，short int 每个位置刚好够放一个hex
  for (i = 0; i < (16 - HUFFMAN_HASH_NBITS); i++)     //  对于一些比较长的编码，会存在队列中，用循环比较的方式识别
    table->slowtable[i][0] = 0;

  /* Build a temp array
   *   huffcode[X] => code used to write vals[X]
   */
  code = 0; // 树从0开始
  hc = huffcode;
  hz = huffsize;
  nbits = *hz; // 因为前几行可能会有0数据长度行的情况，所以前面huffsize 已经保存了第一个非零的行，以此为起点构建
  while (*hz)  // 遍历保存的16位对应长度的数据
  {
    while (*hz == nbits) // 解析出来的前16位hex是代表对应行数据量的大小，在前面构建hz时，就会把同一行，多个数据都赋值位对应行的行数，所以这里需要一次全部拿到对应到行的数据空间
    {
      *hc++ = code++; //  :对同样bit 长度的赋予长度相同的2进制值，这些值即树结构上对应索引值
      hz++;
    }
    code <<= 1; // code切换到下一个码长的范围 |(00),(01)2|(100),(101),(111)3|(1000)(1001)....4|...  赋值遵循C_j=(C_{j-k}+1)<<K
    nbits++;
  }
  // 以上实际已经将所有符号和值一一对应起来，其中code按顺序储存在huffcode,symbol 则隐含在huffsize里面
  /*
   * Build the lookup table, and the slowtable if needed.
   */
  next_free_entry = -1;
  for (i = 0; huffsize[i]; i++) // 遍历所有有长度的行的每个节点，只要存在节点(not null)就不会停止
  {
    val = vals[i];           // 获取每个symbol，即每个会被霍夫曼树映射的原值
    code = huffcode[i];      // symbol 对应的 二进制值
    code_size = huffsize[i]; // symbol所代表的二进制数所用的位数长度

    trace("val=%2.2x code=%8.8x codesize=%2.2d\n", val, code, code_size); // val=symbol(被映射为树的原值) code=二进制代表(树节点) codesize=所使用的二级制长度(树深度)

    // table->code_size 是一个512=2^9的数组，所以可以保存0->2^9数据中的任一位，且能够直接索引
    table->code_size[val] = code_size;   // 开始构建树，先将树中对应到的symble 位置的数据赋予对应的长度值占位，多个不同的symble有可能长度值相同
    if (code_size <= HUFFMAN_HASH_NBITS) // 长度短于9的，直接通过缓存的方式创建映射，空间换时间
    {
      /*
       * Good: val can be put in the lookup table, so fill all value of this
       * column with value val
       *
       */
      /*
        按照如下的结构，lookup数组有512即2^9,为9层的树结构,将二进制最短的数据放在最深的位置，这样二进制短的可以更快(因为数量更多)被匹配到，然后越长的二进制索引则越靠上(类似于升级打怪,到顶部的二进制实际上没有几个，遇到的可能性没有最底下的高)
                  o
               /     \
            o   o   o  o
           / \ / \ / \ / \
          o  o o o o o o  o
          .................
      */
      int repeat = 1UL << (HUFFMAN_HASH_NBITS - code_size);
      code <<= HUFFMAN_HASH_NBITS - code_size;
      while (repeat--)
        table->lookup[code++] = val;
    }
    else
    {
      /* Perhaps sorting the array will be an optimization */
      uint16_t *slowtable = table->slowtable[code_size - HUFFMAN_HASH_NBITS - 1]; // HUFFMAN_HASH_NBITS=9
      while (slowtable[0])                                                        // 只能遍历7次，大于9的只有剩下的7个长度的数据
        slowtable += 2;                                                           // 当前的索引已经被占用，遍历到最后一个没有占用的为止
      slowtable[0] = code;                                                        // CG:有必要为3个值设置如此大的空间吗(256)意义是什么？是不是可以优化掉？
      slowtable[1] = val;
      slowtable[2] = 0;
      /* TODO: NEED TO CHECK FOR AN OVERFLOW OF THE TABLE */
    }
  }
}

static void build_default_huffman_tables(struct jdec_private *priv)
{
  if ((priv->flags & TINYJPEG_FLAGS_MJPEG_TABLE) && priv->default_huffman_table_initialized)
    return;

  build_huffman_table(bits_dc_luminance, val_dc_luminance, &priv->HTDC[0]);
  build_huffman_table(bits_ac_luminance, val_ac_luminance, &priv->HTAC[0]);

  build_huffman_table(bits_dc_chrominance, val_dc_chrominance, &priv->HTDC[1]);
  build_huffman_table(bits_ac_chrominance, val_ac_chrominance, &priv->HTAC[1]);

  priv->default_huffman_table_initialized = 1;
}

/*******************************************************************************
 *
 * Colorspace conversion routine
 *
 *
 * Note:
 * YCbCr is defined per CCIR 601-1, except that Cb and Cr are
 * normalized to the range 0..MAXJSAMPLE rather than -0.5 .. 0.5.
 * The conversion equations to be implemented are therefore
 *      R = Y                + 1.40200 * Cr
 *      G = Y - 0.34414 * Cb - 0.71414 * Cr
 *      B = Y + 1.77200 * Cb
 *
 ******************************************************************************/

static unsigned char clamp(int i)
{
  if (i < 0)
    return 0;
  else if (i > 255)
    return 255;
  else
    return i;
}

/**
 *  YCrCb -> YUV420P (1x1)
 *  .---.
 *  | 1 |
 *  `---'
 */
static void YCrCB_to_YUV420P_1x1(struct jdec_private *priv)
{
  const unsigned char *s, *y;
  unsigned char *p;
  int i, j;

  p = priv->plane[0];
  y = priv->Y;
  for (i = 0; i < 8; i++)
  {
    memcpy(p, y, 8);
    p += priv->width;
    y += 8;
  }

  p = priv->plane[1];
  s = priv->Cb;
  for (i = 0; i < 8; i += 2)
  {
    for (j = 0; j < 8; j += 2, s += 2)
      *p++ = *s;
    s += 8; /* Skip one line */
    p += priv->width / 2 - 4;
  }

  p = priv->plane[2];
  s = priv->Cr;
  for (i = 0; i < 8; i += 2)
  {
    for (j = 0; j < 8; j += 2, s += 2)
      *p++ = *s;
    s += 8; /* Skip one line */
    p += priv->width / 2 - 4;
  }
}

/**
 *  YCrCb -> YUV420P (2x1)
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void YCrCB_to_YUV420P_2x1(struct jdec_private *priv)
{
  unsigned char *p;
  const unsigned char *s, *y1;
  unsigned int i;

  p = priv->plane[0];
  y1 = priv->Y;
  for (i = 0; i < 8; i++)
  {
    memcpy(p, y1, 16);
    p += priv->width;
    y1 += 16;
  }

  p = priv->plane[1];
  s = priv->Cb;
  for (i = 0; i < 8; i += 2)
  {
    memcpy(p, s, 8);
    s += 16; /* Skip one line */
    p += priv->width / 2;
  }

  p = priv->plane[2];
  s = priv->Cr;
  for (i = 0; i < 8; i += 2)
  {
    memcpy(p, s, 8);
    s += 16; /* Skip one line */
    p += priv->width / 2;
  }
}

/**
 *  YCrCb -> YUV420P (1x2)
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void YCrCB_to_YUV420P_1x2(struct jdec_private *priv)
{
  const unsigned char *s, *y;
  unsigned char *p;
  int i, j;

  p = priv->plane[0];
  y = priv->Y;
  for (i = 0; i < 16; i++)
  {
    memcpy(p, y, 8);
    p += priv->width;
    y += 8;
  }

  p = priv->plane[1];
  s = priv->Cb;
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j += 2, s += 2)
      *p++ = *s;
    p += priv->width / 2 - 4;
  }

  p = priv->plane[2];
  s = priv->Cr;
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j += 2, s += 2)
      *p++ = *s;
    p += priv->width / 2 - 4;
  }
}

int chroma_ount = 0;

/**
 *  YCrCb -> YUV420P (2x2)
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
int test = 2;
static void YCrCB_to_YUV420P_2x2(struct jdec_private *priv)
{
  unsigned char *p;
  const unsigned char *s, *y1;
  unsigned int i;

  int diff_w_16x = priv->width % 16, diff_h_16x = priv->height % 16;
  int height_min16x = (priv->height - diff_h_16x);
  int width_min16x = (priv->width - diff_w_16x); // for yuv 420 (4mcu)
  int delete_count = 16 - (priv->width % 16);    //  为了完成16block 而多出来的部分，可以直接丢弃
  int gaped = 0, index = 0, gaped_h = 0;
  unsigned char *y_p = priv->components[0]; // luma 分量所分配的空间地址

  p = priv->plane[0]; // 当前luma 变量已经保存的数据栈顶位置
  index = (p - y_p);  // index 为当前已经保存的luma 数据量

  if ((index - width_min16x) % priv->width == 0 && index != 0 && diff_w_16x != 0)
  {
    gaped = 1; // 当前的p栈顶指向即将为多余数据分配多余16block的地方，即从这里开始，要丢弃掉部分没有用的数据。这是行数据补16block的适配
  }
  if (index >= (height_min16x * priv->width) && priv->height_resized) // 判断当前指针是否位于即将为高度不足16block而补的位置
  {
    // now is the extend row
    gaped_h = diff_h_16x; // 保存多出来的行数
  }
  // printf(" index(%d),gaped_h(%d),height_min16x(%d)\n", index, gaped_h, priv->height * priv->width);
  y1 = priv->Y;
  for (i = 0; i < 16; i++)
  {

    if (gaped_h == 0 || (gaped_h != 0 && i < gaped_h)) // 只保留非补充行的数据
    {
      if (!gaped) //  当没有位于需要补16block的地方时，默认保留所有数据
        memcpy(p, y1, 16);
      else
        memcpy(p, y1, 16 - delete_count);

      p += priv->width; // plane指向最终储存解码数据的component,存完MUC的一行后需要跳到下一行继续存,即+width
      y1 += 16;         // yuv420为了照顾chroma 分量，luma 分量扩展到16*16的大小，跳转下一行即＋16
    }
    else
    {
      // printf(" >>>>>>>now break\n");
      break; // 当遇到第一个为了凑够16block的补充行时，后面的行也都是补充的，可以直接跳过这些行
    }
  }

  p = priv->plane[1]; // chroma u
  s = priv->Cb;
  for (i = 0; i < 8; i++)
  {

    if (gaped_h == 0 || (gaped_h != 0 && i < (gaped_h / 2)))
    {
      if (!gaped)
      {
        memcpy(p, s, 8);
      }
      else
      {
        // printf(" [%s][%d],diff=%d\n", __func__, __LINE__, diff_w_16x / 2);
        memcpy(p, s, diff_w_16x / 2);
      }
      s += 8;
      p += priv->width / 2;
    }
    else
    {
      break;
    }
  }

  p = priv->plane[2]; // chroma v
  s = priv->Cr;
  for (i = 0; i < 8; i++)
  {
    if (gaped_h == 0 || (gaped_h != 0 && i < (gaped_h / 2)))
    {
      if (!gaped)
      {
        memcpy(p, s, 8);
      }
      else
      {
        // printf(" [%s][%d],diff=%d\n", __func__, __LINE__, diff_w_16x / 2);
        memcpy(p, s, diff_w_16x / 2);
      }

      s += 8;
      p += priv->width / 2;
    }
    else
    {
      break;
    }
  }
}

/**
 *  YCrCb -> RGB24 (1x1)
 *  .---.
 *  | 1 |
 *  `---'
 */
static void YCrCB_to_RGB24_1x1(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = priv->width * 3 - 8 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      y = (*Y++) << SCALEBITS;
      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
    }

    p += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/**
 *  YCrCb -> BGR24 (1x1)
 *  .---.
 *  | 1 |
 *  `---'
 */
static void YCrCB_to_BGR24_1x1(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = priv->width * 3 - 8 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      y = (*Y++) << SCALEBITS;
      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
    }

    p += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/**
 *  YCrCb -> RGB24 (2x1)
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void YCrCB_to_RGB24_2x1(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = priv->width * 3 - 16 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      y = (*Y++) << SCALEBITS;
      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);

      y = (*Y++) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
    }

    p += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/*
 *  YCrCb -> BGR24 (2x1)
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void YCrCB_to_BGR24_2x1(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = priv->width * 3 - 16 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      y = (*Y++) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);

      y = (*Y++) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
    }

    p += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/**
 *  YCrCb -> RGB24 (1x2)
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void YCrCB_to_RGB24_1x2(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p, *p2;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  p2 = priv->plane[0] + priv->width * 3;
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = 2 * priv->width * 3 - 8 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      y = (*Y++) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);

      y = (Y[8 - 1]) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);
    }
    Y += 8;
    p += offset_to_next_row;
    p2 += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/*
 *  YCrCb -> BGR24 (1x2)
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void YCrCB_to_BGR24_1x2(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p, *p2;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  p2 = priv->plane[0] + priv->width * 3;
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = 2 * priv->width * 3 - 8 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      y = (*Y++) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);

      y = (Y[8 - 1]) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);
    }
    Y += 8;
    p += offset_to_next_row;
    p2 += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/**
 *  YCrCb -> RGB24 (2x2)
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
static void YCrCB_to_RGB24_2x2(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p, *p2;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  p2 = priv->plane[0] + priv->width * 3;
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = (priv->width * 3 * 2) - 16 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      y = (*Y++) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);

      y = (*Y++) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);

      y = (Y[16 - 2]) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);

      y = (Y[16 - 1]) << SCALEBITS;
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);
    }
    Y += 16;
    p += offset_to_next_row;
    p2 += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/*
 *  YCrCb -> BGR24 (2x2)
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
static void YCrCB_to_BGR24_2x2(struct jdec_private *priv)
{
  const unsigned char *Y, *Cb, *Cr;
  unsigned char *p, *p2;
  int i, j;
  int offset_to_next_row;

#define SCALEBITS 10
#define ONE_HALF (1UL << (SCALEBITS - 1))
#define FIX(x) ((int)((x) * (1UL << SCALEBITS) + 0.5))

  p = priv->plane[0];
  p2 = priv->plane[0] + priv->width * 3;
  Y = priv->Y;
  Cb = priv->Cb;
  Cr = priv->Cr;
  offset_to_next_row = (priv->width * 3 * 2) - 16 * 3;
  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
    {

      int y, cb, cr;
      int add_r, add_g, add_b;
      int r, g, b;

      cb = *Cb++ - 128;
      cr = *Cr++ - 128;
      add_r = FIX(1.40200) * cr + ONE_HALF;
      add_g = -FIX(0.34414) * cb - FIX(0.71414) * cr + ONE_HALF;
      add_b = FIX(1.77200) * cb + ONE_HALF;

      y = (*Y++) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);

      y = (*Y++) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p++ = clamp(r);

      y = (Y[16 - 2]) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);

      y = (Y[16 - 1]) << SCALEBITS;
      b = (y + add_b) >> SCALEBITS;
      *p2++ = clamp(b);
      g = (y + add_g) >> SCALEBITS;
      *p2++ = clamp(g);
      r = (y + add_r) >> SCALEBITS;
      *p2++ = clamp(r);
    }
    Y += 16;
    p += offset_to_next_row;
    p2 += offset_to_next_row;
  }

#undef SCALEBITS
#undef ONE_HALF
#undef FIX
}

/**
 *  YCrCb -> Grey (1x1)
 *  .---.
 *  | 1 |
 *  `---'
 */
static void YCrCB_to_Grey_1x1(struct jdec_private *priv)
{
  const unsigned char *y;
  unsigned char *p;
  unsigned int i;
  int offset_to_next_row;

  p = priv->plane[0];
  y = priv->Y;
  offset_to_next_row = priv->width;

  for (i = 0; i < 8; i++)
  {
    memcpy(p, y, 8);
    y += 8;
    p += offset_to_next_row;
  }
}

/**
 *  YCrCb -> Grey (2x1)
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void YCrCB_to_Grey_2x1(struct jdec_private *priv)
{
  const unsigned char *y;
  unsigned char *p;
  unsigned int i;

  p = priv->plane[0];
  y = priv->Y;

  for (i = 0; i < 8; i++)
  {
    memcpy(p, y, 16);
    y += 16;
    p += priv->width;
  }
}

/**
 *  YCrCb -> Grey (1x2)
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void YCrCB_to_Grey_1x2(struct jdec_private *priv)
{
  const unsigned char *y;
  unsigned char *p;
  unsigned int i;

  p = priv->plane[0];
  y = priv->Y;

  for (i = 0; i < 16; i++)
  {
    memcpy(p, y, 8);
    y += 8;
    p += priv->width;
  }
}

/**
 *  YCrCb -> Grey (2x2)
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
static void YCrCB_to_Grey_2x2(struct jdec_private *priv)
{
  const unsigned char *y;
  unsigned char *p;
  unsigned int i;

  p = priv->plane[0];
  y = priv->Y;

  for (i = 0; i < 16; i++)
  {
    memcpy(p, y, 16);
    y += 16;
    p += priv->width;
  }
}

/*
 * Decode all the 3 components for 1x1
 */
static void decode_MCU_1x1_3planes(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 8);

  // Cb
  process_Huffman_data_unit(priv, cCb);
  IDCT(&priv->component_infos[cCb], priv->Cb, 8);

  // Cr
  process_Huffman_data_unit(priv, cCr);
  IDCT(&priv->component_infos[cCr], priv->Cr, 8);
}

/*
 * Decode a 1x1 directly in 1 color
 */
static void decode_MCU_1x1_1plane(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 8);

  // Cb
  process_Huffman_data_unit(priv, cCb);
  IDCT(&priv->component_infos[cCb], priv->Cb, 8);

  // Cr
  process_Huffman_data_unit(priv, cCr);
  IDCT(&priv->component_infos[cCr], priv->Cr, 8);
}

/*
 * Decode a 2x1
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void decode_MCU_2x1_3planes(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 8, 16);

  // Cb
  process_Huffman_data_unit(priv, cCb);
  IDCT(&priv->component_infos[cCb], priv->Cb, 8);

  // Cr
  process_Huffman_data_unit(priv, cCr);
  IDCT(&priv->component_infos[cCr], priv->Cr, 8);
}

/*
 * Decode a 2x1
 *  .-------.
 *  | 1 | 2 |
 *  `-------'
 */
static void decode_MCU_2x1_1plane(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 8, 16);

  // Cb
  process_Huffman_data_unit(priv, cCb);

  // Cr
  process_Huffman_data_unit(priv, cCr);
}

// tinyjpeg 这份代码暂时只支持以16为倍数的解码，后续将加上逻辑
/*
  1.420的图片,意味着4y1u1v,为了让所有通道的数据量都为64的整数倍,需要有4*64 y 1*64u 1*64v ,
    即需要同步解析4个y数据单元(单个数据单元大小为8*8=64)组成一个单独的MCU(Minimum Coded Unit最小编码单元)
  2.由1可知，针对yuv420，解码一个MCU，需要解码出4*64个y，1*64个u和1*64个v，对于yuv420，其chroma分量是
    使用相邻两行采样的方式，假设第一行采样了u分量，则第二行只采样v分量，这样对于chroma数据可以少一半而又
    不至于影响图片质量
    y u y   y u y   y u y   y u y
    y v y   y v y   y v y   y v y
    ....
    如上，一般8*8的luma 单元，按420只会有16个u单元和16个v单元，这样不利于储存，
    为了便于chroma 分量能够直接成为8*8 矩阵保存,jpeg 在编码过程中，将luma分量使用了
    隔行扫描的方式，sos里保存的数据，按顺序先为解析数据单元1和2的y 数据,按顺序分别为
    数据单元1的1,3,5,7行，数据单元1解析结束后到数据单元2,依旧按顺序1,3,5,7,这样解析
    完sos遇到的第一8*8数据块之后，一半MCU单元的奇数行数据就解析出来了，同样的方法，
    对sos之后的一个64数据解析，得到偶数行数据，两者结合即为MCU上半部分y数据。
    这样对于储存数据来说是方便的，不用解析64个y后把16个v和u变量存起来，可以继续解析，也
    利于避免丢失部分块后颜色完全无法显示正常(按照16u/v要考虑丢失的情况)，当然这仅局限于420和422，
    yuv444则最符合个人认知。

 * Decode a 2x2
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
static void decode_MCU_2x2_3planes(struct jdec_private *priv)
{
  // trace("decode_MCU_2x2_3planes\n");//3 run here
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 16); // (struct component *compptr, uint8_t *output_buf, int stride);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 8, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64 * 2, 16); // 第三个数据单位储存在64*2之后
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64 * 2 + 8, 16);

  // Cb
  process_Huffman_data_unit(priv, cCb);
  IDCT(&priv->component_infos[cCb], priv->Cb, 8);

  // Cr
  process_Huffman_data_unit(priv, cCr);
  IDCT(&priv->component_infos[cCr], priv->Cr, 8);
}

/*
 * Decode a 2x2 directly in GREY format (8bits)
 *  .-------.
 *  | 1 | 2 |
 *  |---+---|
 *  | 3 | 4 |
 *  `-------'
 */
static void decode_MCU_2x2_1plane(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 8, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64 * 2, 16);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64 * 2 + 8, 16);

  // Cb
  process_Huffman_data_unit(priv, cCb);

  // Cr
  process_Huffman_data_unit(priv, cCr);
}

/*
 * Decode a 1x2 mcu
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void decode_MCU_1x2_3planes(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 8);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64, 8);

  // Cb
  process_Huffman_data_unit(priv, cCb);
  IDCT(&priv->component_infos[cCb], priv->Cb, 8);

  // Cr
  process_Huffman_data_unit(priv, cCr);
  IDCT(&priv->component_infos[cCr], priv->Cr, 8);
}

/*
 * Decode a 1x2 mcu
 *  .---.
 *  | 1 |
 *  |---|
 *  | 2 |
 *  `---'
 */
static void decode_MCU_1x2_1plane(struct jdec_private *priv)
{
  // Y
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y, 8);
  process_Huffman_data_unit(priv, cY);
  IDCT(&priv->component_infos[cY], priv->Y + 64, 8);

  // Cb
  process_Huffman_data_unit(priv, cCb);

  // Cr
  process_Huffman_data_unit(priv, cCr);
}

static void print_SOF(const unsigned char *stream)
{
  int width, height, nr_components, precision;
#if DEBUG
  const char *nr_components_to_string[] = {
      "????",
      "Grayscale",
      "????",
      "YCbCr",
      "CYMK"};
#endif

  precision = stream[2];
  height = be16_to_cpu(stream + 3);
  width = be16_to_cpu(stream + 5);
  nr_components = stream[7];

  trace("> SOF marker\n");
  trace("Size:%dx%d nr_components:%d (%s)  precision:%d\n",
        width, height,
        nr_components, nr_components_to_string[nr_components],
        precision);
}

/*******************************************************************************
 *
 * JPEG/JFIF Parsing functions
 *
 * Note: only a small subset of the jpeg file format is supported. No markers,
 * nor progressive stream is supported.
 *
 ******************************************************************************/
/**
 * qtable: 空白的量化表结构
 * ref_table: 来自文件流信息中的实际数据，4*16=64 个量化信息，按照正常的线性排列(左到右，上到下)
 */
static void build_quantization_table(float *qtable, const unsigned char *ref_table)
{
  /* Taken from libjpeg. Copyright Independent JPEG Group's LLM idct.
   * For float AA&N IDCT method, divisors are equal to quantization
   * coefficients scaled by scalefactor[row]*scalefactor[col], where
   *   scalefactor[0] = 1
   *   scalefactor[k] = cos(k*PI/16) * sqrt(2)    for k=1..7
   * We apply a further scale factor of 8.
   * What's actually stored is 1/divisor so that the inner loop can
   * use a multiplication rather than a division.
   */
  int i, j;
  static const double aanscalefactor[8] = {
      1.0, 1.387039845, 1.306562965, 1.175875602, 1.0, 0.785694958, 0.541196100, 0.275899379};
  //  量化矩阵 https://programs.wiki/wiki/jpeg-coding-principle-file-format-and-code-analysis.html
  // https://www.orgleaf.com/4018.html JPEG规定的量化表考虑了人眼视觉特性对不同频率分量的敏感特性：对低频敏感，对高频不敏感，因此对低频数据采用了细量化，对高频数据采用了粗量化。
  const unsigned char *zz = zigzag; //  曲折扫描法

  // :两个for循环做一个8*8=64的量化表
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      *qtable++ = ref_table[*zz++] * aanscalefactor[i] * aanscalefactor[j]; //  使用量化矩阵将源数据的量化表再量化一次,数据源ref_table的默认排列不是zigzag，所以这里也用了zigzag的方法去索引到对应位置的数据
    }
  }
}

/**
 * @brief
 *
 * @param priv jdec结构体指针
 * @param stream 原始图片流指针,初始指向DQT mark后的一块地址，代表DQT信息总长度
 * @return int
 */
static int parse_DQT(struct jdec_private *priv, const unsigned char *stream)
{
  int qi;
  float *table;
  const unsigned char *dqt_block_end;

  trace("> DQT marker\n");
  dqt_block_end = stream + be16_to_cpu(stream);
  stream += 2; /* Skip length */

  while (stream < dqt_block_end)
  {
    qi = *stream++;
#if SANITY_CHECK //  完整性检查
    if (qi >> 4)
      error("16 bits quantization table is not supported\n");
    if (qi > 4) //  当前最多支持3个量化表
      error("No more 4 quantization table is supported (got %d)\n", qi);
#endif
    table = priv->Q_tables[qi]; // :一个三行64列的结构，用于存放量化信息
    build_quantization_table(table, stream);
    stream += 64;
  }
  trace("< DQT marker\n");
  return 0;
}

/**
 * @brief 解析SOF块(SOI->APP0->DQT->SOF)
 *
 * @param priv
 * @param stream
 * @return int
 */
static int parse_SOF(struct jdec_private *priv, const unsigned char *stream)
{
  int i, width, height, nr_components, cid, sampling_factor;
  int Q_table;
  struct component *c;

  trace("> SOF marker\n");
  print_SOF(stream);

  // 宽高计算因为涉及到16位转换，只能通过地址索引
  height = be16_to_cpu(stream + 3); // SOF mark后第4个字节代表高度
  width = be16_to_cpu(stream + 5);  // SOF mark后第5个字节代表宽度
  // component 不需要做hex转换，所以可以通过数组下标直接获取
  nr_components = stream[7]; // SOF mark 后第7位代表nr_components(颜色分量(number of color components))
#if SANITY_CHECK
  if (stream[2] != 8)
    error("Precision other than 8 is not supported\n");
  if (width > JPEG_MAX_WIDTH || height > JPEG_MAX_HEIGHT)
    error("Width and Height (%dx%d) seems suspicious\n", width, height);
  if (nr_components != 3)
    error("We only support YUV images\n");

  priv->height_resized = 0;
  if (height % 16)
  {
    priv->height_resized = 1; // 标记当前图片高度不够MCU分解，需要多一部分MCU
    // error("Height need to be a multiple of 16 (current height is %d)\n", height);
  }

  if (width % 16)
  {
    // error("Width need to be a multiple of 16 (current Width is %d)\n", width);
  }

#endif
  stream += 8;
  for (i = 0; i < nr_components; i++)
  {
    cid = *stream++; // component id号
    sampling_factor = *stream++;
    Q_table = *stream++;
    c = &priv->component_infos[i]; //  遍历可以设置的组件，设置组件id(cid), 采样方式，使用的量化表编号，在sos段中将用于索引(组件id(色彩通道))
#if SANITY_CHECK
    c->cid = cid;
    if (Q_table >= COMPONENTS) // 量化表和组件号是对应的,允许多个factor使用同一个量化表?
      error("Bad Quantization table index (got %d, max allowed %d)\n", Q_table, COMPONENTS - 1);
#endif
    c->Vfactor = sampling_factor & 0xf; // 垂直方向的样本因子(vertical sample factor)
    c->Hfactor = sampling_factor >> 4;  // 水平方向的样本因子(horizontal sample factor)
    c->Q_table = priv->Q_tables[Q_table];
    trace("Component:%d  factor:%dx%d  Quantization table:%d\n",
          cid, c->Hfactor, c->Hfactor, Q_table);
  }
  priv->width = width;
  priv->height = height;

  trace("< SOF marker\n");

  return 0;
}

static int parse_SOS(struct jdec_private *priv, const unsigned char *stream)
{
  unsigned int i, cid, table;
  unsigned int nr_components = stream[2]; // 图像通道数量(Y,U,V)

  trace("> SOS marker\n");

#if SANITY_CHECK
  if (nr_components != 3)
    error("We only support YCbCr image\n");
#endif

  stream += 3;
  // 设定不同通道数据会使用的哈夫曼编码树
  for (i = 0; i < nr_components; i++)
  {
    cid = *stream++;   // component id
    table = *stream++; // 一个hex用于表示对应通道使用的哈曼夫编码表编号
#if SANITY_CHECK
    if ((table & 0xf) >= 4)
      error("We do not support more than 2 AC Huffman table\n");
    if ((table >> 4) >= 4)
      error("We do not support more than 2 DC Huffman table\n");
    if (cid != priv->component_infos[i].cid) //  在SOF中获取到图片通道id(component id),会需要在这里矫正一次
      error("SOS cid order (%d:%d) isn't compatible with the SOF marker (%d:%d)\n",
            i, cid, i, priv->component_infos[i].cid);
    trace("ComponentId:%d  tableAC:%d tableDC:%d,table:%d\n", cid, table & 0xf, table >> 4, table);
#endif
    // 设定组件使用的霍夫曼编码表,AC和DC中分别可以包含多张表
    priv->component_infos[i].AC_table = &priv->HTAC[table & 0xf]; // 低4位是ac对应的表号
    priv->component_infos[i].DC_table = &priv->HTDC[table >> 4];  // 高4位是dc对应的表号
  }
  priv->stream = stream + 3; // 组件对应的编码表和ac，dc信息确认后，将保存sos中开始解析的位置用于decode。
  trace("< SOS marker\n");
  return 0;
}

/**
 * @brief 霍夫曼编码树
 * 获取jpg文件携带的霍夫曼编码
 * @param priv
 * @param stream
 * @return int
 */
static int parse_DHT(struct jdec_private *priv, const unsigned char *stream)
{
  unsigned int count, i;
  unsigned char huff_bits[17];
  int length, index;

  length = be16_to_cpu(stream) - 2; //  长度包含两个hex 值，eg:00 A3 = 163
  stream += 2;                      /* Skip length */

  trace("> DHT marker (length=%d)\n", length);

  while (length > 0)
  {
    index = *stream++; // 高 4 位为 0，表示 DC(直流）哈夫曼表。低 4 位表示哈夫曼表的 ID。

    /* We need to calculate the number of bytes 'vals' will takes */
    huff_bits[0] = 0;
    count = 0;
    for (i = 1; i < 17; i++) // : 刚好遍历16个字节,对应 范式哈夫曼编码 的16个bits
    {
      huff_bits[i] = *stream++; // 按顺序，将最前面的16个hex放到16个bits中，这16个hex数字描述 Code Length 的个数(Number)
      count += huff_bits[i];    // : 总计所有code的数量
    }
#if SANITY_CHECK
    if (count >= HUFFMAN_BITS_SIZE)
      error("No more than %d bytes is allowed to describe a huffman table", HUFFMAN_BITS_SIZE);
    if ((index & 0xf) >= HUFFMAN_TABLES) // 最多支持3个霍夫曼编码树(低4位确认id)
      error("No more than %d Huffman tables is supported (got %d)\n", HUFFMAN_TABLES, index & 0xf);
    trace("Huffman table %s[%d] length=%d\n", (index & 0xf0) ? "AC" : "DC", index & 0xf, count);
#endif

    if (index & 0xf0)                                                   // 高四位判断是DC还是AC，  DC(0)直流分量,AC(1)交流分量
      build_huffman_table(huff_bits, stream, &priv->HTAC[index & 0xf]); // :构建霍夫曼编码表，AC
    else
      build_huffman_table(huff_bits, stream, &priv->HTDC[index & 0xf]); // DC，这里再&上是为了获取到霍夫曼编码树id

    length -= 1;     // 减去头部标识index的长度
    length -= 16;    // 减去16位长度表示所用长度
    length -= count; // 减去symbol所用的长度
    stream += count; // 继续解析下一个霍夫曼编码表,一般包含ac，dc，以及多个不同编码标签的霍夫曼编码表
  }
  trace("< DHT marker\n");
  return 0;
}

/**
 * @brief 解析jpeg DRI(Define restart interval)
 *
 * @param priv
 * @param stream
 * @return int
 */
static int parse_DRI(struct jdec_private *priv, const unsigned char *stream)
{
  unsigned int length;

  trace("> DRI marker\n");

  length = be16_to_cpu(stream);

#if SANITY_CHECK
  if (length != 4)
    error("Length of DRI marker need to be 4\n");
#endif

  priv->restart_interval = be16_to_cpu(stream + 2);

#if DEBUG
  trace("Restart interval = %d\n", priv->restart_interval);
#endif

  trace("< DRI marker\n");

  return 0;
}

static void resync(struct jdec_private *priv)
{
  int i;

  /* Init DC coefficients */
  for (i = 0; i < COMPONENTS; i++)
    priv->component_infos[i].previous_DC = 0;

  priv->reservoir = 0;
  priv->nbits_in_reservoir = 0;
  if (priv->restart_interval > 0)
    priv->restarts_to_go = priv->restart_interval;
  else
    priv->restarts_to_go = -1;
}

static int find_next_rst_marker(struct jdec_private *priv)
{
  int rst_marker_found = 0;
  int marker;
  const unsigned char *stream = priv->stream;

  /* Parse marker */
  while (!rst_marker_found)
  {
    while (*stream++ != 0xff)
    {
      if (stream >= priv->stream_end)
        error("EOF while search for a RST marker.");
    }
    /* Skip any padding ff byte (this is normal) */
    while (*stream == 0xff)
      stream++;

    marker = *stream++;
    if ((RST + priv->last_rst_marker_seen) == marker)
      rst_marker_found = 1;
    else if (marker >= RST && marker <= RST7)
      error("Wrong Reset marker found, abording");
    else if (marker == EOI)
      return 0;
  }
  trace("RST Marker %d found at offset %d\n", priv->last_rst_marker_seen, stream - priv->stream_begin);

  priv->stream = stream;
  priv->last_rst_marker_seen++;
  priv->last_rst_marker_seen &= 7;

  return 0;
}

static int parse_JFIF(struct jdec_private *priv, const unsigned char *stream)
{
  int chuck_len;
  int marker;
  int sos_marker_found = 0;
  int dht_marker_found = 0;
  const unsigned char *next_chunck;

  /* Parse marker */
  while (!sos_marker_found)
  {
    if (*stream++ != 0xff) //  所有块以FF开头
      goto bogus_jpeg_format;
    /* Skip any padding ff byte (this is normal) */
    while (*stream == 0xff) // 当匹配到第一个FF后，忽略后面遇到的其他FF(jpg允许FF后跟无限个FF)
      stream++;

    marker = *stream++; //  无符号字符范围(0-255)刚好用于解析hex
    //  printf(">>>TC(%d)(%d)(%d)\n",stream[0]<<8,stream[0],stream[1]);
    chuck_len = be16_to_cpu(stream);  //  起始字节FF和maker字节后的两个字节为chunk的长度信息，需要将高8为和低8位数据结合到一起计算。这里用到了be16函数，本质是将前一个字节的数据放在高8位(左移)，后一个字节放在低8位最终得到int32数据即为chunk 长度
    next_chunck = stream + chuck_len; //  通过chuck len和当前chunk头的指针地址，获取到下一个chuck的位置

    switch (marker) //  maker 字节区分不同chunk 的作用，jpeg解码chunk顺序(SOI(图片起点),APP0(文件信息),DQT(量化表),SOF0(帧头?),DHT(霍夫曼编码表),SOS(Start of scan,实际的文件信息),EOI(文件末尾))
    {
    case APP0:
      // printf("Now is Application 0\n");          //SOI后的chunk
      break;
    case SOF: // 0xC0   Start of Frame              //解析完DQT后，就到到SOF
      if (parse_SOF(priv, stream) < 0)
        return -1;
      break;
    case DQT: //  0xDB   Define Quantization Table 定义图片的使用的量化表，APP0后的chunk
      if (parse_DQT(priv, stream) < 0)
        return -1;
      break;
    case SOS: // Start of Scan 获取通道(组件)id和对应使用的霍夫曼编码树
      if (parse_SOS(priv, stream) < 0)
        return -1;
      sos_marker_found = 1; // 当发现sos标签时，代表可以开始解码，会跳出chunk的识别循环
      break;
    case DHT: //  0xC4 霍夫曼编码表,一些图片的DHT会在SOF之后，
      if (parse_DHT(priv, stream) < 0)
        return -1;
      dht_marker_found = 1;
      break;
    case DRI:
      if (parse_DRI(priv, stream) < 0)
        return -1;
      break;
    default:
      trace("> Unknown marker %2.2x\n", marker); //  这里会跳过(0xE0和0xFE)直接解析其他的chunk
      break;
    }

    stream = next_chunck;
  }

  if (!dht_marker_found) //  如果前面解析结束了没有找到设定的huffman编码表，则使用默认的
  {
    trace("No Huffman table loaded, using the default one\n");
    build_default_huffman_tables(priv); //  check how to load default
  }

#ifdef SANITY_CHECK // :完整性检查
  if ((priv->component_infos[cY].Hfactor < priv->component_infos[cCb].Hfactor) || (priv->component_infos[cY].Hfactor < priv->component_infos[cCr].Hfactor))
    error("Horizontal sampling factor for Y should be greater than horitontal sampling factor for Cb or Cr\n");
  if ((priv->component_infos[cY].Vfactor < priv->component_infos[cCb].Vfactor) || (priv->component_infos[cY].Vfactor < priv->component_infos[cCr].Vfactor))
    error("Vertical sampling factor for Y should be greater than vertical sampling factor for Cb or Cr\n");
  if ((priv->component_infos[cCb].Hfactor != 1) || (priv->component_infos[cCr].Hfactor != 1) || (priv->component_infos[cCb].Vfactor != 1) || (priv->component_infos[cCr].Vfactor != 1))
    error("Sampling other than 1x1 for Cr and Cb is not supported");
#endif

  return 0;
bogus_jpeg_format:
  trace("Bogus jpeg format\n");
  return -1;
}

/*******************************************************************************
 *
 * Functions exported of the library.
 *
 * Note: Some applications can access directly to internal pointer of the
 * structure. It's is not recommended, but if you have many images to
 * uncompress with the same parameters, some functions can be called to speedup
 * the decoding.
 *
 ******************************************************************************/

/**
 * Allocate a new tinyjpeg decoder object.
 *
 * Before calling any other functions, an object need to be called.
 */
struct jdec_private *tinyjpeg_init(void)
{
  struct jdec_private *priv;

  priv = (struct jdec_private *)calloc(1, sizeof(struct jdec_private));
  if (priv == NULL)
    return NULL;
  return priv;
}

/**
 * Free a tinyjpeg object.
 *
 * No others function can be called after this one.
 */
void tinyjpeg_free(struct jdec_private *priv)
{
  int i;
  for (i = 0; i < COMPONENTS; i++)
  {
    if (priv->components[i])
      free(priv->components[i]);
    priv->components[i] = NULL;
  }
  free(priv);
}

/**
 * Initialize the tinyjpeg object and prepare the decoding of the stream.
 *
 * Check if the jpeg can be decoded with this jpeg decoder.
 * Fill some table used for preprocessing.
 */
int tinyjpeg_parse_header(struct jdec_private *priv, const unsigned char *buf, unsigned int size)
{
  int ret;

  /* Identify the file */
  if ((buf[0] != 0xFF) || (buf[1] != SOI)) // :JPG文件格式以0xFFD8开头
    error("Not a JPG file ?\n");

  priv->stream_begin = buf + 2;
  priv->stream_length = size - 2;
  priv->stream_end = priv->stream_begin + priv->stream_length;

  ret = parse_JFIF(priv, priv->stream_begin);

  return ret;
}

static const decode_MCU_fct decode_mcu_3comp_table[4] = {
    decode_MCU_1x1_3planes,
    decode_MCU_1x2_3planes,
    decode_MCU_2x1_3planes,
    decode_MCU_2x2_3planes,
};

static const decode_MCU_fct decode_mcu_1comp_table[4] = {
    decode_MCU_1x1_1plane,
    decode_MCU_1x2_1plane,
    decode_MCU_2x1_1plane,
    decode_MCU_2x2_1plane,
};

static const convert_colorspace_fct convert_colorspace_yuv420p[4] = {
    YCrCB_to_YUV420P_1x1,
    YCrCB_to_YUV420P_1x2,
    YCrCB_to_YUV420P_2x1,
    YCrCB_to_YUV420P_2x2,
};

static const convert_colorspace_fct convert_colorspace_rgb24[4] = {
    YCrCB_to_RGB24_1x1,
    YCrCB_to_RGB24_1x2,
    YCrCB_to_RGB24_2x1,
    YCrCB_to_RGB24_2x2,
};

static const convert_colorspace_fct convert_colorspace_bgr24[4] = {
    YCrCB_to_BGR24_1x1,
    YCrCB_to_BGR24_1x2,
    YCrCB_to_BGR24_2x1,
    YCrCB_to_BGR24_2x2,
};

static const convert_colorspace_fct convert_colorspace_grey[4] = {
    YCrCB_to_Grey_1x1,
    YCrCB_to_Grey_1x2,
    YCrCB_to_Grey_2x1,
    YCrCB_to_Grey_2x2,
};

/**
 * Decode and convert the jpeg image into @pixfmt@ image
 *
 * Note: components will be automaticaly allocated if no memory is attached.
 */
int tinyjpeg_decode(struct jdec_private *priv, int pixfmt)
{
  unsigned int x, y, xstride_by_mcu, ystride_by_mcu;
  unsigned int bytes_per_blocklines[3], bytes_per_mcu[3];
  decode_MCU_fct decode_MCU;
  const decode_MCU_fct *decode_mcu_table;              // 函数指针,针对不同数据使用不同的解码方式
  const convert_colorspace_fct *colorspace_array_conv; // 针对不同输出的色彩方式使用不同的编码函数(不是encode，只是为了把文件独立分开)
  convert_colorspace_fct convert_to_pixfmt;

  if (setjmp(priv->jump_state))
    return -1;

  /* To keep gcc happy initialize some array */
  bytes_per_mcu[1] = 0; // 对应不同mcu需要采用的步长
  bytes_per_mcu[2] = 0;
  bytes_per_blocklines[1] = 0;
  bytes_per_blocklines[2] = 0;

  decode_mcu_table = decode_mcu_3comp_table; //  togo ...
  switch (pixfmt)
  {
  case TINYJPEG_FMT_YUV420P:
    colorspace_array_conv = convert_colorspace_yuv420p;
    if (priv->components[0] == NULL)
      priv->components[0] = (uint8_t *)malloc(priv->width * priv->height); // 灰度部分，按照像素大小创建一块缓冲。这里把y分量放在前面是为了避免y的内容占用uv 创建出来的空间，而没有出现溢出的情况
    if (priv->components[1] == NULL)
      priv->components[1] = (uint8_t *)malloc(priv->width * priv->height / 4); // uv 部分，在420的情况下4:2:0,即4个灰度1个色彩信息
    if (priv->components[2] == NULL)
      priv->components[2] = (uint8_t *)malloc(priv->width * priv->height / 4);
    bytes_per_blocklines[0] = priv->width;
    bytes_per_blocklines[1] = priv->width / 2; // 这种计算方式会导致出现小数而化整的情况，导致精度减低
    bytes_per_blocklines[2] = priv->width / 2;
    bytes_per_mcu[0] = 8; //  why here is 8:4:4
    bytes_per_mcu[1] = 4;
    bytes_per_mcu[2] = 4;
    break;

  case TINYJPEG_FMT_RGB24:
    colorspace_array_conv = convert_colorspace_rgb24;
    if (priv->components[0] == NULL)
      priv->components[0] = (uint8_t *)malloc(priv->width * priv->height * 3);
    bytes_per_blocklines[0] = priv->width * 3;
    bytes_per_mcu[0] = 3 * 8;
    break;

  case TINYJPEG_FMT_BGR24:
    colorspace_array_conv = convert_colorspace_bgr24;
    if (priv->components[0] == NULL)
      priv->components[0] = (uint8_t *)malloc(priv->width * priv->height * 3);
    bytes_per_blocklines[0] = priv->width * 3;
    bytes_per_mcu[0] = 3 * 8;
    break;

  case TINYJPEG_FMT_GREY:
    decode_mcu_table = decode_mcu_1comp_table;
    colorspace_array_conv = convert_colorspace_grey;
    if (priv->components[0] == NULL)
      priv->components[0] = (uint8_t *)malloc(priv->width * priv->height);
    bytes_per_blocklines[0] = priv->width;
    bytes_per_mcu[0] = 8;
    break;

  default:
    trace("Bad pixel format\n");
    return -1;
  }
  printf(" bytes_per_blocklines[0](%d),bytes_per_blocklines[1](%d),bytes_per_blocklines[2](%d)\n",
         bytes_per_blocklines[0], bytes_per_blocklines[1], bytes_per_blocklines[2]);
  // printf(" [%s][%d],Hfactor(%d),Vfactor(%d)\n",__FUNCTION__,__LINE__,priv->component_infos[cY].Hfactor,priv->component_infos[cY].Vfactor);
  xstride_by_mcu = ystride_by_mcu = 8;
  if ((priv->component_infos[cY].Hfactor | priv->component_infos[cY].Vfactor) == 1) // 判断luma 采样方式,yuv420因为极限压缩chrmoa 分量，实际上可以认为其luma是2*2采样
  {
    decode_MCU = decode_mcu_table[0];             // 不同采样方式使用不同的解码函数
    convert_to_pixfmt = colorspace_array_conv[0]; // 同上，使用指定的颜色解析函数
    trace("Use decode 1x1 sampling\n");
  }
  else if (priv->component_infos[cY].Hfactor == 1)
  {
    decode_MCU = decode_mcu_table[1];
    convert_to_pixfmt = colorspace_array_conv[1];
    ystride_by_mcu = 16;
    trace("Use decode 1x2 sampling (not supported)\n");
  }
  else if (priv->component_infos[cY].Vfactor == 2)
  {
    decode_MCU = decode_mcu_table[3];
    convert_to_pixfmt = colorspace_array_conv[3];
    xstride_by_mcu = 16;
    ystride_by_mcu = 16; // yuv420，使用64*4的做为一个MCU,即16*16的大方块
    trace("Use decode 2x2 sampling\n");
  }
  else
  {
    decode_MCU = decode_mcu_table[2];
    convert_to_pixfmt = colorspace_array_conv[2];
    xstride_by_mcu = 16;
    trace("Use decode 2x1 sampling\n");
  }

  resync(priv); // 初始化

  /* Don't forget to that block can be either 8 or 16 lines */
  bytes_per_blocklines[0] *= ystride_by_mcu; // width*mcu
  if (TINYJPEG_FMT_YUV420P == pixfmt)
  {
    bytes_per_blocklines[1] *= ystride_by_mcu / 2;
    bytes_per_blocklines[2] *= ystride_by_mcu / 2; // this fix func is ugly ,need improve it
  }
  else
  {
    bytes_per_blocklines[1] *= ystride_by_mcu;
    bytes_per_blocklines[2] *= ystride_by_mcu;
  }

  bytes_per_mcu[0] *= xstride_by_mcu / 8;
  bytes_per_mcu[1] *= xstride_by_mcu / 8;
  bytes_per_mcu[2] *= xstride_by_mcu / 8;

  // printf(" priv->width*priv->height(%d),ystride_by_mcu(%d)\n", priv->width * priv->height, ystride_by_mcu);
  // printf(" [%s][%d],bytes_per_mcu[0](%d),bytes_per_mcu[1](%d),bytes_per_mcu[2](%d),priv->restarts_to_go(%d),bytes_per_blocklines[0](%d),bytes_per_blocklines[1](%d)\n",
  //        __FUNCTION__, __LINE__,
  //        bytes_per_mcu[0],
  //        bytes_per_mcu[1],
  //        bytes_per_mcu[2],
  //        priv->restarts_to_go,
  //        bytes_per_blocklines[0], bytes_per_blocklines[1]);

  /* Just the decode the image by macroblock (size is 8x8, 8x16, or 16x16) */
  for (y = 0; y < (priv->height / ystride_by_mcu) + priv->height_resized; y++) // 按照MCU的宽度遍历行，y=row count
  {
    // trace("Decoding row %d\n", y);
    priv->plane[0] = priv->components[0] + (y * bytes_per_blocklines[0]); // components 为最终导出格式的空间
    priv->plane[1] = priv->components[1] + (y * bytes_per_blocklines[1]);
    priv->plane[2] = priv->components[2] + (y * bytes_per_blocklines[2]);
    for (x = 0; x < (priv->width); x += xstride_by_mcu) // 按MCU长度遍历列
    {
      decode_MCU(priv);                   // 实际解码函数和idtc实作
      convert_to_pixfmt(priv);            // 导出为指定格式
      priv->plane[0] += bytes_per_mcu[0]; // 因为是MCU为计算单元大小，但components储存空间是线性且按照图片大小(左上到右下按像素单元)储存，所以这里依旧是以一个MCU为步长做跳转,然后在convert_to_pixfmt()里做换行逻辑
      priv->plane[1] += bytes_per_mcu[1];
      priv->plane[2] += bytes_per_mcu[2];
      if (priv->restarts_to_go > 0)
      {
        priv->restarts_to_go--;
        if (priv->restarts_to_go == 0)
        {
          priv->stream -= (priv->nbits_in_reservoir / 8);
          resync(priv);
          if (find_next_rst_marker(priv) < 0)
            return -1;
        }
      }
    }
  }

  trace("Input file size: %d\n", priv->stream_length + 2);
  trace("Input bytes actually read: %d\n", priv->stream - priv->stream_begin + 2);

  return 0;
}

const char *tinyjpeg_get_errorstring(struct jdec_private *priv)
{
  /* FIXME: the error string must be store in the context */
  priv = priv;
  return error_string;
}

void tinyjpeg_get_size(struct jdec_private *priv, unsigned int *width, unsigned int *height)
{
  *width = priv->width;
  *height = priv->height;
}

int tinyjpeg_get_components(struct jdec_private *priv, unsigned char **components)
{
  int i;
  for (i = 0; i < COMPONENTS; i++) //  这里的判断会有问题，暂时摘掉会好
  {
    if (priv->components[i] != NULL)
    {
      components[i] = priv->components[i];
    }
    else
    {
      break;
    }
  }
  return 0;
}

int tinyjpeg_set_components(struct jdec_private *priv, unsigned char **components, unsigned int ncomponents)
{
  unsigned int i;
  if (ncomponents > COMPONENTS)
    ncomponents = COMPONENTS;
  for (i = 0; i < ncomponents; i++)
    priv->components[i] = components[i];
  return 0;
}

int tinyjpeg_set_flags(struct jdec_private *priv, int flags)
{
  int oldflags = priv->flags;
  priv->flags = flags;
  return oldflags;
}
