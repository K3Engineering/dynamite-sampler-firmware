#ifndef BOARD_ID_h
#define BOARD_ID_h

#include <stddef.h>

#include <esp_err.h>

#include "board_cfg.h"

// Buffer (incl NUL) for the Factory `board_model` value.
constexpr size_t BOARD_IDENTITY_NAME_MAX = 16;

// DIS identity reported when the board has no usable identity in flash.
constexpr char BOARD_IDENTITY_NONE[] = "UNCONFIGURED";
constexpr char BOARD_IDENTITY_MARKETING_NONE[] = "Dynamite Sampler (unconfigured)";

// Inits both NVS partitions, then reads Factory `board_model` and selects the
// board config. Absent or unknown identity is not an error: it boots into
// safe mode. Returns an error only on storage failure.
esp_err_t initBoardIdentity();

// Selected board config; nullptr in safe mode (no/unknown identity).
const BoardCfg *boardCfg();

#endif // BOARD_ID_h
