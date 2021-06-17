#ifndef __WAV_HEAD_H__
#define __WAV_HEAD_H__

typedef struct waveHeader {
  char cid[4];    // RIFF
  uint32_t fsize;
  char format[4]; // WAVE
  char scid1[4];    // 'fmt '
  uint32_t scid1_size;
  uint16_t audio_format; // 1 = pcm
  uint16_t channels;
  uint32_t sample_rate;
  uint32_t byte_rate;
  uint16_t block_align; // 8,16,32 num_chan*bpp/8
  uint16_t bits_per_sample; // 8, 16,i24, 32
  char scid2[4];  // data
  uint32_t scid2_size;

} waveHeader;




const char *headByteLabels[44] = {
"CID",
"CID",
"CID",
"CID",
"CID_SIZE",
"CID_SIZE",
"CID_SIZE",
"CID_SIZE",
"FORMAT",
"FORMAT",
"FORMAT",
"FORMAT",
"SCID1",
"SCID1",
"SCID1",
"SCID1",
"SCID1_SIZE",
"SCID1_SIZE",
"SCID1_SIZE",
"SCID1_SIZE",
"AUDIO_FORMAT",
"AUDIO_FORMAT",
"NUM_CHAN",
"NUM_CHAN",
"SAMPLE_RATE",
"SAMPLE_RATE",
"SAMPLE_RATE",
"SAMPLE_RATE",
"BYTE_RATE",
"BYTE_RATE",
"BYTE_RATE",
"BYTE_RATE",
"BLOCK_ALIGN",
"BLOCK_ALIGN",
"BITS_PER_SAMPLE",
"BITS_PER_SAMPLE",
"SCID2_ID",
"SCID2_ID",
"SCID2_ID",
"SCID2_ID",
"SCID2_SIZE",
"SCID2_SIZE",
"SCID2_SIZE",
"SCID2_SIZE"
};



#endif // __WAV_HEAD_H__
