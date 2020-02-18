/*
    This example code is in the Public Domain (or CC0 licensed, at your option.)

    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdint.h>

void spdif_init(int rate);
void spdif_write(const void *src, size_t size);
void spdif_set_sample_rates(int rate);
