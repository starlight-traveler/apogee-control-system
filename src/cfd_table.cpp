#include "cfd_table.h"

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <vector>

#include "data_logger.h"

namespace {

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
    acc->acs.push_back(values[0]);
    acc->atk.push_back(values[1]);
    acc->mach.push_back(values[2]);
    acc->sawData = true;
    return true;
}

void SortUnique(std::vector<double> &values) {
    std::sort(values.begin(), values.end());
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
        ctx->missing++;
        return true;
    }
    const int index = (i * ctx->atkCount + j) * ctx->machCount + k;
    ctx->storage->axial[index] = values[3];
    ctx->storage->normal[index] = values[4];
    return true;
}

}  // namespace

bool CfdTableLoadFromSd(const char *path, CfdTableStorage *storage, bool logSerial) {
    if (storage == nullptr || path == nullptr) {
        return false;
    }

    AxisAccumulator axes;
    axes.acs.reserve(5200);
    axes.atk.reserve(5200);
    axes.mach.reserve(5200);
    if (!DataLoggerReadTextFile(path, &CollectAxisLine, &axes) || !axes.sawData) {
        if (logSerial && Serial) {
            Serial.println("Failed to read CFD table.");
        }
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
        if (logSerial && Serial) {
            Serial.println("CFD table is empty or invalid.");
        }
        storage->loaded = false;
        return false;
    }

    static TableStorage tableStorage;
    tableStorage.acs = std::move(axes.acs);
    tableStorage.atk = std::move(axes.atk);
    tableStorage.mach = std::move(axes.mach);
    tableStorage.axial.assign(total, NAN);
    tableStorage.normal.assign(total, NAN);

    FillContext fill;
    fill.storage = &tableStorage;
    fill.acsCount = acsCount;
    fill.atkCount = atkCount;
    fill.machCount = machCount;

    if (!DataLoggerReadTextFile(path, &FillTableLine, &fill)) {
        if (logSerial && Serial) {
            Serial.println("Failed to load CFD table data.");
        }
        storage->loaded = false;
        return false;
    }

    storage->table.acsAnglesDeg = tableStorage.acs.data();
    storage->table.atkAnglesDeg = tableStorage.atk.data();
    storage->table.machNumbers = tableStorage.mach.data();
    storage->table.axialForces = tableStorage.axial.data();
    storage->table.normalForces = tableStorage.normal.data();
    storage->table.acsCount = acsCount;
    storage->table.atkCount = atkCount;
    storage->table.machCount = machCount;
    storage->loaded = true;

    if (logSerial && Serial) {
        Serial.print("Loaded CFD table: ");
        Serial.print(acsCount);
        Serial.print(" x ");
        Serial.print(atkCount);
        Serial.print(" x ");
        Serial.print(machCount);
        Serial.println(".");
        if (fill.missing > 0) {
            Serial.print("CFD table missing entries: ");
            Serial.println(fill.missing);
        }
    }

    return true;
}
