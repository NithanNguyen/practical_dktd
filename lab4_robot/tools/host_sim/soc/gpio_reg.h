// Gia lap thanh ghi GPIO cho ban bien dich tren PC.
#pragma once
#include <cstdint>
#define GPIO_OUT_W1TS_REG 1
#define GPIO_OUT_W1TC_REG 2
#define GPIO_IN1_REG 3
uint32_t sim_reg_read(int reg);
void sim_reg_write(int reg, uint32_t v);
#define REG_READ(r) sim_reg_read(r)
#define REG_WRITE(r, v) sim_reg_write((r), (v))
