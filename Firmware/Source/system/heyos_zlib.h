
#pragma once

#include <rtthread.h>
#include <stdio.h>

int heyos_zlib_decompress_data2data(uint8_t * src, uint8_t * dest, uint32_t src_len);
int heyos_zlib_decompress_data2file(uint8_t * src, char *des_file_name, uint32_t src_len);
int heyos_zlib_decompress_file2file(char *src_file_name, char *des_file_name);
bool is_gzipfile(char* des);
