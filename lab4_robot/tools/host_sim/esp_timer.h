// Gia lap esp_timer cho ban bien dich tren PC.
#pragma once
#include <cstdint>
typedef void (*esp_timer_cb_t)(void *arg);
typedef struct { esp_timer_cb_t callback; void *arg; int dispatch_method; const char *name; bool skip_unhandled_events; } esp_timer_create_args_t;
typedef struct sim_timer *esp_timer_handle_t;
int64_t esp_timer_get_time();
int esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out);
int esp_timer_start_periodic(esp_timer_handle_t t, uint64_t period_us);
