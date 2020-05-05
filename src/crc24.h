#pragma once
#include <stdint.h>
#include <stddef.h>

uint32_t crc24(size_t len, const uint8_t *data, uint32_t crc);
