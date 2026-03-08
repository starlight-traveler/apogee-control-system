#pragma once

#include <cstdint>
#include <iomanip>
#include <iostream>

class SerialStub {
  public:
    void begin(unsigned long) {}

    template <typename T>
    void print(const T &value) {
        std::cout << value;
    }

    template <typename T>
    void println(const T &value) {
        std::cout << value << std::endl;
    }

    void println() { std::cout << std::endl; }

    void print(float value, int decimals) { PrintWithPrecision(value, decimals, false); }
    void print(double value, int decimals) { PrintWithPrecision(value, decimals, false); }
    void println(float value, int decimals) { PrintWithPrecision(value, decimals, true); }
    void println(double value, int decimals) { PrintWithPrecision(value, decimals, true); }

    explicit operator bool() const { return enabled_; }

    void setEnabled(bool enabled) { enabled_ = enabled; }

  private:
    template <typename T>
    void PrintWithPrecision(T value, int decimals, bool newline) {
        if (decimals >= 0) {
            std::streamsize previousPrecision = std::cout.precision();
            std::ios_base::fmtflags previousFlags = std::cout.flags();
            std::cout.setf(std::ios::fixed, std::ios::floatfield);
            std::cout << std::setprecision(decimals) << value;
            std::cout.flags(previousFlags);
            std::cout << std::setprecision(previousPrecision);
        } else {
            std::cout << value;
        }
        if (newline) {
            std::cout << std::endl;
        }
    }

    bool enabled_ = true;
};

extern SerialStub Serial;

inline void delay(unsigned long) {}
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}

#define HIGH 0x1
#define LOW 0x0
#define OUTPUT 0x1
#define LED_BUILTIN 13
