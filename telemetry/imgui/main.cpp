#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits.h>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sys/wait.h>
#include <signal.h>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "../../include/telemetry_packet.h"

namespace {

struct SharedTelemetry {
    telemetry::PacketV1 latest{};
    uint64_t packetsReceived = 0;
    uint64_t packetsDropped = 0;
    uint32_t lastSequence = 0;
    bool hasPacket = false;
    std::chrono::steady_clock::time_point lastRx = std::chrono::steady_clock::time_point::min();
};

struct ServoCalibrationPoint {
    float angleDeg;
    int topPwmUs;
    int bottomPwmUs;
};

constexpr std::array<ServoCalibrationPoint, 20> kServoCalibrationTable = {{
    {0.0f, 1090, 1967},  {3.2f, 1122, 1935},  {6.3f, 1154, 1903},  {9.5f, 1186, 1871},
    {12.6f, 1218, 1839}, {15.8f, 1250, 1807}, {18.9f, 1282, 1775}, {22.1f, 1315, 1742},
    {25.3f, 1347, 1710}, {28.4f, 1379, 1678}, {31.6f, 1411, 1646}, {34.7f, 1443, 1614},
    {37.9f, 1475, 1582}, {41.1f, 1507, 1550}, {44.2f, 1539, 1518}, {47.4f, 1571, 1486},
    {50.5f, 1603, 1454}, {53.7f, 1636, 1421}, {56.8f, 1668, 1389}, {60.0f, 1700, 1355},
}};

float ClampFloat(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

float SnapToServoLookupAngle(float angleDeg) {
    const float clamped = ClampFloat(angleDeg, 0.0f, 60.0f);
    float bestAngle = kServoCalibrationTable[0].angleDeg;
    float bestDistance = std::fabs(clamped - bestAngle);
    for (const ServoCalibrationPoint &point : kServoCalibrationTable) {
        const float distance = std::fabs(clamped - point.angleDeg);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestAngle = point.angleDeg;
        }
    }
    return bestAngle;
}

std::string TrimString(const std::string &value) {
    size_t start = 0;
    while (start < value.size() && std::isspace(static_cast<unsigned char>(value[start])) != 0) {
        start++;
    }
    size_t end = value.size();
    while (end > start && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
        end--;
    }
    return value.substr(start, end - start);
}

std::string ToLowerString(std::string value) {
    for (char &ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::vector<std::string> TokenizeWords(const std::string &text) {
    std::vector<std::string> tokens;
    std::string current;
    current.reserve(16);
    for (char ch : text) {
        const unsigned char uch = static_cast<unsigned char>(ch);
        if (std::isalnum(uch) != 0) {
            current.push_back(static_cast<char>(std::tolower(uch)));
        } else if (!current.empty()) {
            tokens.push_back(current);
            current.clear();
        }
    }
    if (!current.empty()) {
        tokens.push_back(current);
    }
    return tokens;
}

bool ContainsAnyToken(const std::vector<std::string> &tokens, const std::vector<std::string> &needles) {
    for (const std::string &token : tokens) {
        for (const std::string &needle : needles) {
            if (token == needle) {
                return true;
            }
        }
    }
    return false;
}

std::optional<int> WordToSmallNumber(const std::string &word) {
    static const std::unordered_map<std::string, int> mapping = {
        {"zero", 0}, {"oh", 0}, {"o", 0},
        {"one", 1}, {"won", 1},
        {"two", 2}, {"to", 2}, {"too", 2},
        {"three", 3}, {"tree", 3}, {"free", 3},
        {"four", 4}, {"for", 4}, {"fore", 4},
        {"five", 5},
        {"six", 6},
        {"seven", 7},
        {"eight", 8}, {"ate", 8},
        {"nine", 9},
        {"ten", 10},
        {"eleven", 11},
        {"twelve", 12},
        {"thirteen", 13},
        {"fourteen", 14},
        {"fifteen", 15},
        {"sixteen", 16},
        {"seventeen", 17},
        {"eighteen", 18},
        {"nineteen", 19},
        {"twenty", 20},
        {"thirty", 30}, {"therty", 30}, {"thirdy", 30}, {"dirty", 30},
        {"forty", 40}, {"fourty", 40},
        {"fifty", 50},
        {"sixty", 60}
    };
    auto it = mapping.find(word);
    if (it == mapping.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<int> ParseAngleFromTokens(const std::vector<std::string> &tokens) {
    for (const std::string &token : tokens) {
        bool allDigits = !token.empty();
        for (char ch : token) {
            if (!std::isdigit(static_cast<unsigned char>(ch))) {
                allDigits = false;
                break;
            }
        }
        if (allDigits) {
            return std::atoi(token.c_str());
        }
    }

    for (size_t i = 0; i + 1 < tokens.size(); ++i) {
        const std::optional<int> first = WordToSmallNumber(tokens[i]);
        const std::optional<int> second = WordToSmallNumber(tokens[i + 1]);
        if (!first.has_value() || !second.has_value()) {
            continue;
        }

        if (first.value() >= 20 && first.value() <= 60 && (first.value() % 10) == 0 &&
            second.value() >= 0 && second.value() <= 9) {
            return first.value() + second.value();
        }

        if (first.value() >= 0 && first.value() <= 9 && second.value() >= 0 && second.value() <= 9) {
            return first.value() * 10 + second.value();
        }
    }

    for (const std::string &token : tokens) {
        const std::optional<int> value = WordToSmallNumber(token);
        if (value.has_value()) {
            return value;
        }
    }

    return std::nullopt;
}

struct ParsedVoiceCommand {
    bool setAuto = false;
    float angleDeg = 0.0f;
    std::string transcript;
};

std::optional<ParsedVoiceCommand> ParseVoiceCommand(const std::string &rawText) {
    const std::string text = ToLowerString(rawText);
    const std::vector<std::string> tokens = TokenizeWords(text);
    const bool hasWakeWord = ContainsAnyToken(tokens, {"acs", "apogee"});
    const bool hasMoveVerb = ContainsAnyToken(tokens, {"actuate", "move", "set", "deploy", "extend", "position"});
    const bool hasAngleHint = ContainsAnyToken(tokens, {"degree", "degrees", "deg", "angle"});
    const bool hasAutoWord = ContainsAnyToken(tokens, {"auto", "automatic"});
    const bool hasDisableWord =
        ContainsAnyToken(tokens, {"return", "back", "disable", "off", "stop", "cancel", "release"});

    const bool commandContext = hasWakeWord || hasMoveVerb || hasAngleHint;
    if (!commandContext && !(hasAutoWord && hasDisableWord)) {
        return std::nullopt;
    }

    ParsedVoiceCommand command{};
    command.transcript = rawText;
    if (hasAutoWord && hasDisableWord) {
        command.setAuto = true;
        return command;
    }

    const std::optional<int> parsedAngle = ParseAngleFromTokens(tokens);
    if (!parsedAngle.has_value()) {
        if (ContainsAnyToken(tokens, {"full", "max", "maximum"})) {
            command.angleDeg = 60.0f;
            return command;
        }
        if (ContainsAnyToken(tokens, {"half", "middle", "mid"})) {
            command.angleDeg = 30.0f;
            return command;
        }
        return std::nullopt;
    }

    command.angleDeg = ClampFloat(static_cast<float>(parsedAngle.value()), 0.0f, 60.0f);
    return command;
}

std::string ResolveVoiceScriptPath(const char *argv0) {
    const char *envScript = std::getenv("ACS_VOICE_SCRIPT");
    if (envScript != nullptr && envScript[0] != '\0' && access(envScript, R_OK) == 0) {
        return std::string(envScript);
    }

    std::vector<std::string> candidates;
    candidates.emplace_back("voice_listener.py");

    if (argv0 != nullptr && argv0[0] != '\0') {
        const std::filesystem::path argPath(argv0);
        if (argPath.has_parent_path()) {
            candidates.push_back((argPath.parent_path() / "voice_listener.py").string());
        }
    }

#if defined(__linux__)
    char exePath[PATH_MAX];
    const ssize_t exeLen = readlink("/proc/self/exe", exePath, sizeof(exePath) - 1);
    if (exeLen > 0) {
        exePath[exeLen] = '\0';
        const std::filesystem::path selfPath(exePath);
        candidates.push_back((selfPath.parent_path() / "voice_listener.py").string());
    }
#endif

    for (const std::string &candidate : candidates) {
        if (access(candidate.c_str(), R_OK) == 0) {
            return candidate;
        }
    }

    return "voice_listener.py";
}

class VoiceCommandReceiver {
  public:
    ~VoiceCommandReceiver() { Stop(); }

    bool Start(const char *scriptPath) {
        if (running_.load()) {
            return true;
        }
        if (scriptPath == nullptr || scriptPath[0] == '\0' || access(scriptPath, R_OK) != 0) {
            SetError("voice script not found (set ACS_VOICE_SCRIPT or place voice_listener.py near app)");
            return false;
        }

        int pipeFds[2];
        if (pipe(pipeFds) != 0) {
            SetError("voice pipe() failed");
            return false;
        }

        const pid_t childPid = fork();
        if (childPid < 0) {
            close(pipeFds[0]);
            close(pipeFds[1]);
            SetError("voice fork() failed");
            return false;
        }

        if (childPid == 0) {
            dup2(pipeFds[1], STDOUT_FILENO);
            dup2(pipeFds[1], STDERR_FILENO);
            close(pipeFds[0]);
            close(pipeFds[1]);

            const char *existingModelPath = std::getenv("VOSK_MODEL_PATH");
            if (existingModelPath == nullptr || existingModelPath[0] == '\0') {
                const std::filesystem::path scriptFile(scriptPath);
                const std::filesystem::path modelPath =
                    scriptFile.parent_path() / "models" / "vosk-model-en-us-0.22";
                const std::string modelPathString = modelPath.string();
                setenv("VOSK_MODEL_PATH", modelPathString.c_str(), 0);
            }

            const char *overridePython = std::getenv("ACS_VOICE_PYTHON");
            if (overridePython != nullptr && overridePython[0] != '\0') {
                execl(overridePython, overridePython, scriptPath, (char *)nullptr);
            }

            const char *venvPython = ".venv/bin/python3";
            if (access(venvPython, X_OK) == 0) {
                execl(venvPython, venvPython, scriptPath, (char *)nullptr);
            }

            execlp("python3", "python3", scriptPath, (char *)nullptr);
            _exit(127);
        }

        close(pipeFds[1]);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lastError_.clear();
            latestTranscript_.clear();
            latestAcceptedCommand_.clear();
            hasPendingCommand_ = false;
        }
        readFd_ = pipeFds[0];
        childPid_ = childPid;
        running_.store(true);
        worker_ = std::thread([this]() { ReadLoop(); });
        return true;
    }

    void Stop() {
        const bool wasRunning = running_.exchange(false);

        if (wasRunning && childPid_ > 0) {
            kill(childPid_, SIGTERM);
        }

        if (worker_.joinable()) {
            worker_.join();
        }

        if (childPid_ > 0) {
            int status = 0;
            waitpid(childPid_, &status, 0);
            childPid_ = -1;
        }
    }

    bool Running() const { return running_.load(); }

    std::string LastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastError_;
    }

    std::string LatestTranscript() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latestTranscript_;
    }

    std::string LatestAcceptedCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latestAcceptedCommand_;
    }

    bool ConsumePendingCommand(ParsedVoiceCommand *outCommand) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasPendingCommand_) {
            return false;
        }
        *outCommand = pendingCommand_;
        hasPendingCommand_ = false;
        return true;
    }

  private:
    void SetError(const std::string &text) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastError_ = text;
    }

    void ReadLoop() {
        FILE *stream = fdopen(readFd_, "r");
        if (stream == nullptr) {
            SetError("voice fdopen() failed");
            return;
        }

        char lineBuffer[512];
        while (running_.load() && std::fgets(lineBuffer, sizeof(lineBuffer), stream) != nullptr) {
            std::string line = TrimString(std::string(lineBuffer));
            if (line.empty()) {
                continue;
            }

            if (line.rfind("ERROR:", 0) == 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                lastError_ = line;
                continue;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            latestTranscript_ = line;
            const std::optional<ParsedVoiceCommand> parsed = ParseVoiceCommand(line);
            if (parsed.has_value()) {
                pendingCommand_ = parsed.value();
                hasPendingCommand_ = true;
                if (parsed->setAuto) {
                    latestAcceptedCommand_ = "Voice: return to auto";
                } else {
                    char temp[96];
                    std::snprintf(temp, sizeof(temp), "Voice: actuate %.0f deg", parsed->angleDeg);
                    latestAcceptedCommand_ = temp;
                }
            }
        }

        std::fclose(stream);
        running_.store(false);
    }

    mutable std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    int readFd_ = -1;
    pid_t childPid_ = -1;
    std::string lastError_;
    std::string latestTranscript_;
    std::string latestAcceptedCommand_;
    ParsedVoiceCommand pendingCommand_{};
    bool hasPendingCommand_ = false;
};

class UdpReceiver {
  public:
    UdpReceiver(uint16_t port, const char *teensyIp, uint16_t teensyPort)
        : port_(port), teensyPort_(teensyPort) {
        std::memset(&teensyAddr_, 0, sizeof(teensyAddr_));
        teensyAddr_.sin_family = AF_INET;
        teensyAddr_.sin_port = htons(teensyPort_);
        if (inet_pton(AF_INET, teensyIp, &teensyAddr_.sin_addr) == 1) {
            heartbeatEnabled_ = true;
        }
    }

    ~UdpReceiver() { Stop(); }

    bool Start() {
        if (running_.load()) {
            return true;
        }
        running_.store(true);
        worker_ = std::thread([this]() { Run(); });
        return true;
    }

    void Stop() {
        if (!running_.exchange(false)) {
            return;
        }
        if (socketFd_ >= 0) {
            close(socketFd_);
            socketFd_ = -1;
        }
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    SharedTelemetry Snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return shared_;
    }

    void SetActuationOverride(bool enabled, float angleDeg) {
        std::lock_guard<std::mutex> lock(commandMutex_);
        const float snappedAngleDeg = enabled ? SnapToServoLookupAngle(angleDeg) : 0.0f;
        const bool changed = (manualActuationOverride_ != enabled) ||
                             (std::fabs(manualActuationAngleDeg_ - snappedAngleDeg) > 1.0e-3f);
        manualActuationOverride_ = enabled;
        manualActuationAngleDeg_ = snappedAngleDeg;
        if (changed) {
            sendActuationImmediately_ = true;
        }
    }

    void SetTelemetryStreamingEnabled(bool enabled) {
        std::lock_guard<std::mutex> lock(commandMutex_);
        if (telemetryStreamingEnabled_ == enabled) {
            return;
        }
        telemetryStreamingEnabled_ = enabled;
        if (enabled) {
            heartbeatEnabled_ = true;
            disconnectPacketsRemaining_ = 0;
            return;
        }

        manualActuationOverride_ = false;
        manualActuationAngleDeg_ = 0.0f;
        disconnectPacketsRemaining_ = 5;
    }

    std::string LastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastError_;
    }

    bool HeartbeatEnabled() const { return heartbeatEnabled_; }

  private:
    void SetError(const char *text) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastError_ = text;
    }

    void Run() {
        socketFd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketFd_ < 0) {
            SetError("socket() failed");
            return;
        }

        const int reuse = 1;
        setsockopt(socketFd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        const int rcvbuf = 1 << 20;
        setsockopt(socketFd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port_);

        if (bind(socketFd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
            SetError("bind() failed; is port already in use?");
            return;
        }

        while (running_.load()) {
            const auto now = std::chrono::steady_clock::now();
            if (heartbeatEnabled_ && telemetryStreamingEnabled_ &&
                (now - lastHeartbeatSent_) >= std::chrono::milliseconds(200)) {
                telemetry::HeartbeatV1 heartbeat{};
                sendto(socketFd_,
                       &heartbeat,
                       sizeof(heartbeat),
                       0,
                       reinterpret_cast<const sockaddr *>(&teensyAddr_),
                       sizeof(teensyAddr_));
                lastHeartbeatSent_ = now;
            }
            bool sendActuationNow = false;
            {
                std::lock_guard<std::mutex> lock(commandMutex_);
                sendActuationNow = sendActuationImmediately_;
                if (sendActuationImmediately_) {
                    sendActuationImmediately_ = false;
                }
            }
            if (heartbeatEnabled_ &&
                (sendActuationNow || (now - lastActuationCommandSent_) >= std::chrono::milliseconds(100))) {
                telemetry::ActuationCommandV1 command{};
                {
                    std::lock_guard<std::mutex> lock(commandMutex_);
                    command.mode = manualActuationOverride_ ? telemetry::kActuationModeManual
                                                            : telemetry::kActuationModeAuto;
                    command.angleDeg = manualActuationAngleDeg_;
                }
                sendto(socketFd_,
                       &command,
                       sizeof(command),
                       0,
                       reinterpret_cast<const sockaddr *>(&teensyAddr_),
                       sizeof(teensyAddr_));
                lastActuationCommandSent_ = now;
            }
            if (heartbeatEnabled_ &&
                (telemetryStreamingEnabled_ || disconnectPacketsRemaining_ > 0) &&
                (now - lastTelemetryControlSent_) >= std::chrono::milliseconds(250)) {
                telemetry::TelemetryControlV1 control{};
                {
                    std::lock_guard<std::mutex> lock(commandMutex_);
                    control.telemetryEnabled = telemetryStreamingEnabled_ ? 1u : 0u;
                }
                sendto(socketFd_,
                       &control,
                       sizeof(control),
                       0,
                       reinterpret_cast<const sockaddr *>(&teensyAddr_),
                       sizeof(teensyAddr_));
                lastTelemetryControlSent_ = now;
                if (!telemetryStreamingEnabled_ && disconnectPacketsRemaining_ > 0) {
                    --disconnectPacketsRemaining_;
                }
            }

            fd_set readSet;
            FD_ZERO(&readSet);
            FD_SET(socketFd_, &readSet);

            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 100000;

            const int ready = select(socketFd_ + 1, &readSet, nullptr, nullptr, &timeout);
            if (ready <= 0) {
                continue;
            }

            telemetry::PacketV1 packet{};
            sockaddr_in src{};
            socklen_t srcLen = sizeof(src);
            const ssize_t n = recvfrom(socketFd_,
                                       &packet,
                                       sizeof(packet),
                                       0,
                                       reinterpret_cast<sockaddr *>(&src),
                                       &srcLen);
            if (n != static_cast<ssize_t>(sizeof(packet))) {
                continue;
            }

            if (packet.magic != telemetry::kPacketMagic || packet.version != telemetry::kPacketVersion ||
                packet.size != sizeof(telemetry::PacketV1)) {
                continue;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            if (shared_.hasPacket) {
                const uint32_t expected = shared_.lastSequence + 1u;
                if (packet.sequence != expected) {
                    if (packet.sequence > expected) {
                        shared_.packetsDropped += static_cast<uint64_t>(packet.sequence - expected);
                    } else {
                        shared_.packetsDropped += 1;
                    }
                }
            }
            shared_.latest = packet;
            shared_.lastSequence = packet.sequence;
            shared_.packetsReceived++;
            shared_.hasPacket = true;
            shared_.lastRx = std::chrono::steady_clock::now();
        }
    }

    uint16_t port_ = 0;
    mutable std::mutex mutex_;
    SharedTelemetry shared_;
    std::string lastError_;
    std::atomic<bool> running_{false};
    std::thread worker_;
    int socketFd_ = -1;
    sockaddr_in teensyAddr_{};
    uint16_t teensyPort_ = 0;
    bool heartbeatEnabled_ = false;
    std::chrono::steady_clock::time_point lastHeartbeatSent_ = std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point lastActuationCommandSent_ = std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point lastTelemetryControlSent_ = std::chrono::steady_clock::time_point::min();
    mutable std::mutex commandMutex_;
    bool manualActuationOverride_ = false;
    float manualActuationAngleDeg_ = 0.0f;
    bool sendActuationImmediately_ = false;
    bool telemetryStreamingEnabled_ = true;
    uint8_t disconnectPacketsRemaining_ = 0;
};

const char *FlightStatusName(uint8_t value) {
    switch (value) {
        case 0:
            return "ground";
        case 1:
            return "burn";
        case 2:
            return "coast";
        case 3:
            return "overshoot";
        case 4:
            return "descent";
        default:
            return "unknown";
    }
}

ImVec4 FlightStatusColor(uint8_t value) {
    switch (value) {
        case 0:
            return ImVec4(0.55f, 0.72f, 0.98f, 1.0f);
        case 1:
            return ImVec4(1.00f, 0.58f, 0.18f, 1.0f);
        case 2:
            return ImVec4(0.45f, 0.82f, 0.55f, 1.0f);
        case 3:
            return ImVec4(0.95f, 0.34f, 0.34f, 1.0f);
        case 4:
            return ImVec4(0.65f, 0.62f, 0.98f, 1.0f);
        default:
            return ImVec4(0.70f, 0.70f, 0.70f, 1.0f);
    }
}

float Lerp(float a, float b, float t) { return a + (b - a) * ClampFloat(t, 0.0f, 1.0f); }

float Norm3(const float v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

template <size_t N>
struct RingSeries {
    std::array<float, N> values{};
    int count = 0;
    int offset = 0;

    void Add(float value) {
        values[offset] = value;
        offset = (offset + 1) % static_cast<int>(N);
        if (count < static_cast<int>(N)) {
            count++;
        }
    }

    bool Empty() const { return count <= 0; }

    void Range(float *outMin, float *outMax) const {
        if (Empty()) {
            *outMin = 0.0f;
            *outMax = 1.0f;
            return;
        }
        float minValue = values[0];
        float maxValue = values[0];
        for (int i = 1; i < count; ++i) {
            minValue = std::min(minValue, values[i]);
            maxValue = std::max(maxValue, values[i]);
        }
        if (std::fabs(maxValue - minValue) < 1e-4f) {
            minValue -= 0.5f;
            maxValue += 0.5f;
        }
        *outMin = minValue;
        *outMax = maxValue;
    }
};

struct DashboardState {
    RingSeries<300> altitudeAglFeet;
    RingSeries<300> verticalVelocity;
    RingSeries<300> accelMagnitude;
    RingSeries<300> servoCommand;
    RingSeries<300> servoEffective;
    bool hasLastSequence = false;
    uint32_t lastSequence = 0;
    float servoGaugeAnimated = 0.0f;
    float streamHealthAnimated = 0.0f;
    float dropRateAnimated = 0.0f;
};

void ApplyTelemetryTheme() {
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 12.0f;
    style.ChildRounding = 10.0f;
    style.FrameRounding = 8.0f;
    style.PopupRounding = 8.0f;
    style.ScrollbarRounding = 8.0f;
    style.GrabRounding = 8.0f;
    style.WindowPadding = ImVec2(14.0f, 12.0f);
    style.FramePadding = ImVec2(10.0f, 6.0f);
    style.ItemSpacing = ImVec2(10.0f, 8.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;

    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.08f, 0.11f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.82f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.96f);
    colors[ImGuiCol_Border] = ImVec4(0.18f, 0.22f, 0.30f, 0.45f);
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.58f, 0.64f, 0.72f, 1.00f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.16f, 0.19f, 0.25f, 0.90f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.23f, 0.28f, 0.37f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.28f, 0.34f, 0.45f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.12f, 0.16f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.13f, 0.19f, 0.25f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.18f, 0.25f, 0.35f, 0.90f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.24f, 0.34f, 0.47f, 0.92f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.30f, 0.40f, 0.54f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.23f, 0.39f, 0.59f, 0.85f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.46f, 0.67f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.20f, 0.33f, 0.50f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.53f, 0.76f, 0.98f, 0.95f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.68f, 0.84f, 1.00f, 1.00f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.66f, 0.86f, 0.98f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.22f, 0.29f, 0.40f, 0.75f);
    colors[ImGuiCol_PlotLines] = ImVec4(0.33f, 0.82f, 0.74f, 1.00f);
    colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.59f, 0.93f, 0.84f, 1.00f);
}

void DrawBackdrop() {
    ImDrawList *drawList = ImGui::GetBackgroundDrawList();
    const ImVec2 min = ImGui::GetMainViewport()->Pos;
    const ImVec2 max = ImVec2(min.x + ImGui::GetMainViewport()->Size.x, min.y + ImGui::GetMainViewport()->Size.y);
    drawList->AddRectFilledMultiColor(min,
                                      max,
                                      IM_COL32(8, 14, 22, 255),
                                      IM_COL32(9, 18, 28, 255),
                                      IM_COL32(16, 24, 36, 255),
                                      IM_COL32(6, 10, 18, 255));
    drawList->AddCircleFilled(ImVec2(min.x + 220.0f, min.y + 120.0f), 160.0f, IM_COL32(35, 70, 120, 35), 64);
    drawList->AddCircleFilled(ImVec2(max.x - 180.0f, max.y - 90.0f), 210.0f, IM_COL32(20, 120, 95, 30), 64);
}

void DrawMetricCard(const char *label, const char *value, const char *subLabel, const ImVec4 &accent, float width) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.11f, 0.14f, 0.18f, 0.90f));
    ImGui::BeginChild(label, ImVec2(width, 96.0f), true);
    ImDrawList *drawList = ImGui::GetWindowDrawList();
    const ImVec2 min = ImGui::GetWindowPos();
    const ImVec2 max = ImVec2(min.x + ImGui::GetWindowSize().x, min.y + ImGui::GetWindowSize().y);
    drawList->AddRectFilled(ImVec2(min.x, min.y), ImVec2(min.x + 4.0f, max.y), ImGui::GetColorU32(accent), 3.0f);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
    ImGui::TextUnformatted(label);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
    ImGui::PushStyleColor(ImGuiCol_Text, accent);
    ImGui::SetWindowFontScale(1.15f);
    ImGui::TextUnformatted(value);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
    ImGui::TextDisabled("%s", subLabel);
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

template <size_t N>
void DrawSeriesPlot(const char *title, const RingSeries<N> &series, const char *overlay, const ImVec2 &size) {
    if (series.Empty()) {
        ImGui::TextDisabled("%s", title);
        ImGui::Dummy(size);
        return;
    }
    float minY = 0.0f;
    float maxY = 0.0f;
    series.Range(&minY, &maxY);
    ImGui::TextUnformatted(title);
    ImGui::PlotLines(("##" + std::string(title)).c_str(),
                     series.values.data(),
                     series.count,
                     series.offset,
                     overlay,
                     minY,
                     maxY,
                     size);
}

void DrawVectorRow(const char *label, const float v[3], const char *units) {
    ImGui::Text("%s", label);
    ImGui::SameLine(170.0f);
    ImGui::Text("%.2f  %.2f  %.2f %s", v[0], v[1], v[2], units);
}

}  // namespace

int main(int argc, char **argv) {
    uint16_t port = 5005;
    const char *teensyIp = "192.168.4.1";
    uint16_t teensyPort = 5006;
    if (argc >= 2) {
        const long parsed = std::strtol(argv[1], nullptr, 10);
        if (parsed > 0 && parsed <= 65535) {
            port = static_cast<uint16_t>(parsed);
        }
    }
    if (argc >= 3) {
        teensyIp = argv[2];
    }
    if (argc >= 4) {
        const long parsed = std::strtol(argv[3], nullptr, 10);
        if (parsed > 0 && parsed <= 65535) {
            teensyPort = static_cast<uint16_t>(parsed);
        }
    }

    if (!glfwInit()) {
        return 1;
    }

    const char *glslVersion = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow *window = glfwCreateWindow(1280, 720, "Teensy Telemetry Receiver", nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    ApplyTelemetryTheme();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glslVersion);

    UdpReceiver receiver(port, teensyIp, teensyPort);
    receiver.Start();
    const std::string voiceScriptPath = ResolveVoiceScriptPath(argv[0]);
    bool manualActuationOverride = false;
    float manualActuationAngleDeg = 0.0f;
    bool telemetryStreamingEnabled = true;
    bool voiceListenEnabled = false;
    VoiceCommandReceiver voiceReceiver;
    DashboardState dashboard{};

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const SharedTelemetry snap = receiver.Snapshot();
        ParsedVoiceCommand voiceCommand{};
        if (voiceReceiver.ConsumePendingCommand(&voiceCommand)) {
            if (voiceCommand.setAuto) {
                manualActuationOverride = false;
                manualActuationAngleDeg = 0.0f;
            } else {
                manualActuationOverride = true;
                manualActuationAngleDeg = SnapToServoLookupAngle(voiceCommand.angleDeg);
            }
        }

        const auto now = std::chrono::steady_clock::now();
        float msSinceRx = -1.0f;
        if (snap.hasPacket && snap.lastRx != std::chrono::steady_clock::time_point::min()) {
            msSinceRx =
                std::chrono::duration<float, std::milli>(now - snap.lastRx).count();
        }

        if (snap.hasPacket && (!dashboard.hasLastSequence || dashboard.lastSequence != snap.latest.sequence)) {
            const telemetry::PacketV1 &p = snap.latest;
            dashboard.altitudeAglFeet.Add(p.altitudeAglFeet);
            dashboard.verticalVelocity.Add(p.stateVelocity[2]);
            dashboard.accelMagnitude.Add(Norm3(p.sensorAccelIcm));
            dashboard.servoCommand.Add(p.servoCommandDeg);
            dashboard.servoEffective.Add(p.servoEffectiveDeg);
            dashboard.lastSequence = snap.latest.sequence;
            dashboard.hasLastSequence = true;
        }

        const float dt = io.DeltaTime > 0.0f ? io.DeltaTime : (1.0f / 60.0f);
        const float healthTarget = (msSinceRx < 0.0f) ? 0.0f : ClampFloat(1.0f - (msSinceRx / 1200.0f), 0.0f, 1.0f);
        dashboard.streamHealthAnimated = Lerp(dashboard.streamHealthAnimated, healthTarget, dt * 7.0f);
        dashboard.servoGaugeAnimated = Lerp(dashboard.servoGaugeAnimated, manualActuationAngleDeg / 60.0f, dt * 6.0f);
        const double packetTotal = static_cast<double>(snap.packetsReceived + snap.packetsDropped);
        const float dropRate = (packetTotal > 0.0) ? static_cast<float>(snap.packetsDropped / packetTotal) : 0.0f;
        dashboard.dropRateAnimated = Lerp(dashboard.dropRateAnimated, dropRate, dt * 4.0f);

        DrawBackdrop();
        ImGui::SetNextWindowPos(ImGui::GetMainViewport()->Pos);
        ImGui::SetNextWindowSize(ImGui::GetMainViewport()->Size);
        ImGuiWindowFlags rootFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                     ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
        ImGui::Begin("Telemetry Dashboard", nullptr, rootFlags);

        ImGui::Text("Teensy Flight Telemetry");
        ImGui::SameLine();
        ImGui::TextDisabled("listening %u  |  target %s:%u", port, teensyIp, teensyPort);
        ImGui::Separator();

        const float totalCardWidth = ImGui::GetContentRegionAvail().x;
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const float cardWidth = (totalCardWidth - (3.0f * spacing)) * 0.25f;
        char statusValue[64];
        std::snprintf(statusValue, sizeof(statusValue), "%s", snap.hasPacket ? "STREAMING" : "WAITING");
        DrawMetricCard("Receiver", statusValue, receiver.HeartbeatEnabled() ? "heartbeat online" : "invalid target IP", ImVec4(0.36f, 0.77f, 0.56f, 1.0f), cardWidth);
        ImGui::SameLine();
        char freshnessValue[64];
        std::snprintf(freshnessValue, sizeof(freshnessValue), "%.0f%%", dashboard.streamHealthAnimated * 100.0f);
        char freshnessSub[64];
        std::snprintf(freshnessSub, sizeof(freshnessSub), msSinceRx >= 0.0f ? "latest %.1f ms ago" : "no packets yet", msSinceRx);
        DrawMetricCard("Link Health", freshnessValue, freshnessSub, ImVec4(0.32f, 0.78f, 0.94f, 1.0f), cardWidth);
        ImGui::SameLine();
        char packetValue[64];
        std::snprintf(packetValue, sizeof(packetValue), "%llu", static_cast<unsigned long long>(snap.packetsReceived));
        char packetSub[80];
        std::snprintf(packetSub, sizeof(packetSub), "dropped %.0f%% (%llu)", dashboard.dropRateAnimated * 100.0f, static_cast<unsigned long long>(snap.packetsDropped));
        DrawMetricCard("Packets", packetValue, packetSub, ImVec4(0.88f, 0.63f, 0.24f, 1.0f), cardWidth);
        ImGui::SameLine();
        if (snap.hasPacket) {
            char phaseSub[64];
            std::snprintf(phaseSub, sizeof(phaseSub), "seq %u", snap.latest.sequence);
            DrawMetricCard("Flight Phase", FlightStatusName(snap.latest.flightStatus), phaseSub, FlightStatusColor(snap.latest.flightStatus), cardWidth);
        } else {
            DrawMetricCard("Flight Phase", "unknown", "waiting for telemetry", ImVec4(0.65f, 0.65f, 0.70f, 1.0f), cardWidth);
        }

        const std::string error = receiver.LastError();
        if (!error.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Socket error: %s", error.c_str());
        }

        ImGui::Separator();
        if (ImGui::BeginTable("dashboard_split", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            ImGui::BeginChild("left_column", ImVec2(0.0f, 0.0f), false);
            if (snap.hasPacket) {
                const telemetry::PacketV1 &p = snap.latest;
                const bool hasPadAltitude = (p.flags & telemetry::kFlagHasPadAltitude) != 0;
                char overlayAgl[64];
                if (hasPadAltitude) {
                    std::snprintf(overlayAgl, sizeof(overlayAgl), "latest %.1f ft AGL", p.altitudeAglFeet);
                } else {
                    std::snprintf(overlayAgl, sizeof(overlayAgl), "pad altitude unavailable");
                }
                DrawSeriesPlot("Altitude (AGL)", dashboard.altitudeAglFeet, overlayAgl, ImVec2(-1.0f, 115.0f));
                char overlayVel[64];
                std::snprintf(overlayVel, sizeof(overlayVel), "vertical %.2f m/s", p.stateVelocity[2]);
                DrawSeriesPlot("Vertical Velocity", dashboard.verticalVelocity, overlayVel, ImVec2(-1.0f, 115.0f));
                char overlayAccel[64];
                std::snprintf(overlayAccel, sizeof(overlayAccel), "ICM mag %.2f m/s^2", Norm3(p.sensorAccelIcm));
                DrawSeriesPlot("Acceleration Magnitude", dashboard.accelMagnitude, overlayAccel, ImVec2(-1.0f, 115.0f));
            } else {
                DrawSeriesPlot("Altitude (AGL)", dashboard.altitudeAglFeet, "no data", ImVec2(-1.0f, 115.0f));
                DrawSeriesPlot("Vertical Velocity", dashboard.verticalVelocity, "no data", ImVec2(-1.0f, 115.0f));
                DrawSeriesPlot("Acceleration Magnitude", dashboard.accelMagnitude, "no data", ImVec2(-1.0f, 115.0f));
            }
            ImGui::EndChild();

            ImGui::TableNextColumn();
            ImGui::BeginChild("right_column", ImVec2(0.0f, 0.0f), false);
            ImGui::Text("Actuation Control");
            if (!telemetryStreamingEnabled) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Disconnect Telemetry")) {
                telemetryStreamingEnabled = false;
            }
            if (!telemetryStreamingEnabled) {
                ImGui::EndDisabled();
            }
            ImGui::TextDisabled("%s", telemetryStreamingEnabled ? "telemetry streaming enabled"
                                                                : "telemetry disabled on Teensy until reboot");
            ImGui::Separator();
            const bool manualOverrideChanged = ImGui::Checkbox("Manual flap override", &manualActuationOverride);
            float requestedManualAngleDeg = manualActuationAngleDeg;
            const bool sliderChanged =
                ImGui::SliderFloat("Manual angle (deg)", &requestedManualAngleDeg, 0.0f, 60.0f, "%.1f");
            if (sliderChanged) {
                manualActuationAngleDeg = SnapToServoLookupAngle(requestedManualAngleDeg);
                manualActuationOverride = true;
            } else {
                manualActuationAngleDeg = SnapToServoLookupAngle(manualActuationAngleDeg);
            }
            if (ImGui::Button("Deploy 30 deg")) {
                manualActuationOverride = true;
                manualActuationAngleDeg = SnapToServoLookupAngle(30.0f);
            }
            ImGui::SameLine();
            if (ImGui::Button("Full 60 deg")) {
                manualActuationOverride = true;
                manualActuationAngleDeg = SnapToServoLookupAngle(60.0f);
            }
            if (ImGui::Button("Return To Auto")) {
                manualActuationOverride = false;
                manualActuationAngleDeg = 0.0f;
            }
            if (manualOverrideChanged && manualActuationOverride) {
                manualActuationAngleDeg = SnapToServoLookupAngle(manualActuationAngleDeg);
            }
            ImGui::ProgressBar(dashboard.servoGaugeAnimated, ImVec2(-1.0f, 8.0f), "");
            ImGui::Text("Commanded mode: %s", manualActuationOverride ? "manual override" : "auto");
            ImGui::TextDisabled("Manual setpoint %.1f deg (lookup snapped)", manualActuationAngleDeg);

            ImGui::Separator();
            bool voiceToggle = voiceListenEnabled;
            if (ImGui::Checkbox("Voice listen (wake: ACS/Apogee)", &voiceToggle)) {
                if (voiceToggle) {
                    voiceListenEnabled = voiceReceiver.Start(voiceScriptPath.c_str());
                } else {
                    voiceReceiver.Stop();
                    voiceListenEnabled = false;
                }
            }
            ImGui::Text("Voice status: %s", voiceReceiver.Running() ? "listening" : "stopped");
            const std::string voiceError = voiceReceiver.LastError();
            if (!voiceError.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", voiceError.c_str());
            } else {
                ImGui::TextDisabled("Voice commands (wake word optional):");
                ImGui::BulletText("\"move 30\", \"set angle 45\", \"actuate sixty\"");
                ImGui::BulletText("\"thirty\", \"three zero\", \"full\", \"half\"");
                ImGui::BulletText("\"return to auto\", \"auto off\", \"disable automatic\"");
            }
            const std::string transcript = voiceReceiver.LatestTranscript();
            if (!transcript.empty()) {
                ImGui::TextWrapped("Heard: %s", transcript.c_str());
            }
            const std::string accepted = voiceReceiver.LatestAcceptedCommand();
            if (!accepted.empty()) {
                ImGui::TextColored(ImVec4(0.50f, 0.90f, 0.62f, 1.0f), "%s", accepted.c_str());
            }

            receiver.SetActuationOverride(manualActuationOverride, manualActuationAngleDeg);
            receiver.SetTelemetryStreamingEnabled(telemetryStreamingEnabled);

            if (snap.hasPacket) {
                const telemetry::PacketV1 &p = snap.latest;
                const bool hasState = (p.flags & telemetry::kFlagHasFilteredState) != 0;
                const bool flightManualOverride = (p.flags & telemetry::kFlagManualActuationOverride) != 0;

                ImGui::Separator();
                ImGui::Text("Primary Telemetry");
                ImGui::Text("Uptime: %u ms", p.uptimeMs);
                ImGui::Text("Sensor timestamp: %.3f s", p.sensorTimestamp);
                ImGui::Text("Sensor altitude: %.2f ft", p.sensorAltitudeFeet);
                ImGui::Text("Servo command/effective: %.2f / %.2f deg", p.servoCommandDeg, p.servoEffectiveDeg);
                ImGui::Text("Flight computer mode: %s", flightManualOverride ? "manual override" : "auto");
                ImGui::TextDisabled("Commanded mode: %s", manualActuationOverride ? "manual override" : "auto");
                DrawSeriesPlot("Servo Command", dashboard.servoCommand, "deg", ImVec2(-1.0f, 84.0f));
                DrawSeriesPlot("Servo Effective", dashboard.servoEffective, "deg", ImVec2(-1.0f, 84.0f));

                if (ImGui::CollapsingHeader("Sensor Vectors", ImGuiTreeNodeFlags_DefaultOpen)) {
                    DrawVectorRow("Accel BNO XYZ", p.sensorAccelBno, "m/s^2");
                    DrawVectorRow("Accel ICM XYZ", p.sensorAccelIcm, "m/s^2");
                    DrawVectorRow("Gyro XYZ", p.sensorGyro, "rad/s");
                    ImGui::Text("Quaternion");
                    ImGui::SameLine(170.0f);
                    ImGui::Text("%.3f  %.3f  %.3f  %.3f",
                                p.sensorQuaternion[0],
                                p.sensorQuaternion[1],
                                p.sensorQuaternion[2],
                                p.sensorQuaternion[3]);
                    ImGui::Text("Has quaternion: %s", p.sensorHasQuaternion ? "true" : "false");
                    ImGui::Text("ICM quaternion");
                    ImGui::SameLine(170.0f);
                    ImGui::Text("%.3f  %.3f  %.3f  %.3f",
                                p.sensorIcmQuaternion[0],
                                p.sensorIcmQuaternion[1],
                                p.sensorIcmQuaternion[2],
                                p.sensorIcmQuaternion[3]);
                    ImGui::Text("Has ICM quaternion: %s", p.sensorHasIcmQuaternion ? "true" : "false");
                    DrawVectorRow("ICM Yaw/Pitch/Roll", p.sensorIcmYprDeg, "deg");
                    ImGui::Text("Has ICM YPR: %s", p.sensorHasIcmYpr ? "true" : "false");
                }

                if (ImGui::CollapsingHeader("Filtered State", ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Text("Status: %s", hasState ? "valid" : "not available");
                    if (hasState) {
                        ImGui::Text("State time: %.3f s", p.stateTime);
                        DrawVectorRow("Position XYZ", p.statePosition, "m");
                        DrawVectorRow("Velocity XYZ", p.stateVelocity, "m/s");
                        DrawVectorRow("Accel XYZ", p.stateAcceleration, "m/s^2");
                        DrawVectorRow("Inertial Accel", p.stateInertialAcceleration, "m/s^2");
                        ImGui::Text("Zenith: %.3f rad", p.stateZenith);
                        ImGui::Text("Apogee estimate: %.2f m", p.stateApogeeEstimate);
                    }
                }
            } else {
                ImGui::Separator();
                ImGui::TextDisabled("No telemetry packet has arrived yet.");
            }
            ImGui::EndChild();
            ImGui::EndTable();
        }

        ImGui::End();

        ImGui::Render();
        int displayW = 0;
        int displayH = 0;
        glfwGetFramebufferSize(window, &displayW, &displayH);
        glViewport(0, 0, displayW, displayH);
        glClearColor(0.03f, 0.04f, 0.06f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    receiver.Stop();
    voiceReceiver.Stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
