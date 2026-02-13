#pragma once

#include "apogee_model.h"

struct CfdTableStorage {
    ApogeeForceTable table;
    bool loaded = false;
};

// Loads the CFD force table from the SD card. Intended to run once at setup.
// Returns true on success and populates storage.table.
bool CfdTableLoadFromSd(const char *path, CfdTableStorage *storage, bool logSerial);
