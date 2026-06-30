#ifndef ERRORS_H
#define ERRORS_H

#include <stdbool.h>
#include <stdint.h>

void errors_init(void);
void errors_set(uint8_t error_code, bool visible);
void errors_set_slave(uint8_t slave_id, uint8_t error_code, bool visible);
void errors_clear_visible(void);
bool errors_has_stored(void);
bool errors_is_visible(void);
uint8_t errors_code(void);
void errors_update(uint32_t now_ms);

#endif