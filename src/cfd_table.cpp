#include "cfd_table.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "data_logger.h"
#include "serial_logging.h"

namespace {

/*
 * The CFD CSV is stored as sparse rows:
 *
 *   ACS angle, angle of attack, Mach, axial force, normal force
 *
 * The predictor wants dense 3D arrays for fast trilinear interpolation. Loading
 * therefore happens in two passes: collect sorted axes first, then fill every
 * cell. Missing cells are rejected because one NaN in the table can poison the
 * apogee integration.
 */

struct AxisAccumulator {
    std::vector<double> acs;
    std::vector<double> atk;
    std::vector<double> mach;
    bool sawData = false;
};

struct TableStorage {
    std::vector<double> acs;
    std::vector<double> atk;
    std::vector<double> mach;
    std::vector<double> axial;
    std::vector<double> normal;
};

bool ParseRow(const char *line, double *out, int count) {
    if (line == nullptr) {
        return false;
    }
    const char *ptr = line;
    for (int i = 0; i < count; ++i) {
        // strtod advances `end`; if it does not move, this column is not numeric.
        char *end = nullptr;
        const double value = strtod(ptr, &end);
        if (end == ptr) {
            return false;
        }
        out[i] = value;
        ptr = end;
        while (*ptr == ',' || *ptr == ' ' || *ptr == '\t') {
            ++ptr;
        }
    }
    return true;
}

bool CollectAxisLine(const char *line, void *context) {
    AxisAccumulator *acc = static_cast<AxisAccumulator *>(context);
    double values[5];
    if (!ParseRow(line, values, 5)) {
        return true;
    }
    // First pass only collects axis values. The dense table cannot be sized
    // until we know every unique ACS, AoA, and Mach coordinate in the file.
    acc->acs.push_back(values[0]);
    acc->atk.push_back(values[1]);
    acc->mach.push_back(values[2]);
    acc->sawData = true;
    return true;
}

void SortUnique(std::vector<double> &values) {
    std::sort(values.begin(), values.end());
    // CFD rows may arrive in any order; sorted unique axes are what the
    // predictor interpolation code expects.
    values.erase(std::unique(values.begin(), values.end()), values.end());
}

int FindIndex(const std::vector<double> &values, double value) {
    auto it = std::lower_bound(values.begin(), values.end(), value);
    if (it == values.end() || *it != value) {
        return -1;
    }
    return static_cast<int>(it - values.begin());
}

struct FillContext {
    TableStorage *storage = nullptr;
    int acsCount = 0;
    int atkCount = 0;
    int machCount = 0;
    int missing = 0;
};

bool FillTableLine(const char *line, void *context) {
    FillContext *ctx = static_cast<FillContext *>(context);
    double values[5];
    if (!ParseRow(line, values, 5)) {
        return true;
    }
    const int i = FindIndex(ctx->storage->acs, values[0]);
    const int j = FindIndex(ctx->storage->atk, values[1]);
    const int k = FindIndex(ctx->storage->mach, values[2]);
    if (i < 0 || j < 0 || k < 0) {
        // This should not happen after the axis pass; count it so the table is
        // rejected instead of silently leaving a hole.
        ctx->missing++;
        return true;
    }
    // Flatten (acs, atk, mach) into one contiguous vector used by the predictor.
    const int index = (i * ctx->atkCount + j) * ctx->machCount + k;
    ctx->storage->axial[index] = values[3];
    ctx->storage->normal[index] = values[4];
    return true;
}

}  // namespace

bool CfdTableLoadFromSd(const char *path, CfdTableStorage *storage) {
    if (storage == nullptr || path == nullptr) {
        return false;
    }

    AxisAccumulator axes;
    axes.acs.reserve(5200);
    axes.atk.reserve(5200);
    axes.mach.reserve(5200);
    if (!DataLoggerReadTextFile(path, &CollectAxisLine, &axes) || !axes.sawData) {
        LOG_PRINTLN("Failed to read CFD table.");
        storage->loaded = false;
        return false;
    }

    SortUnique(axes.acs);
    SortUnique(axes.atk);
    SortUnique(axes.mach);

    const int acsCount = static_cast<int>(axes.acs.size());
    const int atkCount = static_cast<int>(axes.atk.size());
    const int machCount = static_cast<int>(axes.mach.size());
    const int total = acsCount * atkCount * machCount;

    if (acsCount < 2 || atkCount < 2 || machCount < 2 || total <= 0) {
        LOG_PRINTLN("CFD table is empty or invalid.");
        storage->loaded = false;
        return false;
    }

    static TableStorage tableStorage;
    tableStorage.acs = std::move(axes.acs);
    tableStorage.atk = std::move(axes.atk);
    tableStorage.mach = std::move(axes.mach);
    tableStorage.axial.assign(total, NAN);
    tableStorage.normal.assign(total, NAN);

    // Second pass fills the dense force grid. Starting with NaN makes missing
    // cells easy to detect after loading.
    FillContext fill;
    fill.storage = &tableStorage;
    fill.acsCount = acsCount;
    fill.atkCount = atkCount;
    fill.machCount = machCount;

    if (!DataLoggerReadTextFile(path, &FillTableLine, &fill)) {
        LOG_PRINTLN("Failed to load CFD table data.");
        storage->loaded = false;
        return false;
    }

    int invalidCells = fill.missing;
    for (int index = 0; index < total; ++index) {
        if (!std::isfinite(tableStorage.axial[index]) ||
            !std::isfinite(tableStorage.normal[index])) {
            ++invalidCells;
        }
    }
    if (invalidCells > 0) {
        // A single NaN can poison interpolation and produce a bad apogee
        // estimate, so the whole table is refused if any cell is incomplete.
        LOG_PRINT("CFD table missing/invalid entries: ");
        LOG_PRINTLN(invalidCells);
        storage->loaded = false;
        return false;
    }

    // The vectors are static so the predictor can safely hold raw pointers
    // after this loader returns.
    storage->table.acsAnglesDeg = tableStorage.acs.data();
    storage->table.atkAnglesDeg = tableStorage.atk.data();
    storage->table.machNumbers = tableStorage.mach.data();
    storage->table.axialForces = tableStorage.axial.data();
    storage->table.normalForces = tableStorage.normal.data();
    storage->table.acsCount = acsCount;
    storage->table.atkCount = atkCount;
    storage->table.machCount = machCount;
    storage->loaded = true;

    LOG_PRINT("Loaded CFD table: ");
    LOG_PRINT(acsCount);
    LOG_PRINT(" x ");
    LOG_PRINT(atkCount);
    LOG_PRINT(" x ");
    LOG_PRINT(machCount);
    LOG_PRINTLN(".");

    return true;
}
