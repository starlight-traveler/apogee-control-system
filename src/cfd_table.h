#pragma once

#include "apogee_model.h"

/**
 * @brief owns the loaded cfd table pointers and loaded-state flag.
 *
 * the actual arrays are backed by static storage inside `cfd_table.cpp`, so
 * this wrapper is small enough to keep globally in `main.cpp`.
 */
struct CfdTableStorage {
    // Interpolation metadata and pointers into the backing static arrays.
    ApogeeForceTable table;
    // False means the predictor must fall back to its simpler aero model.
    bool loaded = false;
};

/**
 * @brief loads the cfd force table from the sd card.
 *
 * @param path sd-card path to the csv table.
 * @param storage output table wrapper to populate.
 * @return true when every axis and force cell was loaded and finite.
 *
 * the predictor is only allowed to use the table if every cell in the 3d grid
 * has a real force value. missing cells would make interpolation unreliable.
 */
bool CfdTableLoadFromSd(const char *path, CfdTableStorage *storage);
