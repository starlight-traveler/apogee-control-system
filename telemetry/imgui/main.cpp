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
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits.h>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <GL/gl.h>
#endif

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
#include <curl/curl.h>
#include <portaudio.h>
#include <whisper.h>
#endif

#include "../../include/telemetry_packet.h"

namespace {

const char *MainQuaternionSourceName(uint8_t value) {
    switch (value) {
        case 1:
            return "BNO";
        case 2:
            return "ICM";
        case 3:
            return "LSM";
        case 4:
            return "Blended";
        default:
            return "None";
    }
}

struct SharedTelemetry {
    telemetry::PacketV1 latest{};
    telemetry::SettingsSnapshotV1 settings{};
    uint64_t packetsReceived = 0;
    uint64_t packetsDropped = 0;
    uint32_t lastSequence = 0;
    bool hasPacket = false;
    bool hasSettings = false;
    std::chrono::steady_clock::time_point lastRx = std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point lastSettingsRx = std::chrono::steady_clock::time_point::min();
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

const char *SettingsResultName(uint8_t result) {
    switch (result) {
        case telemetry::kSettingsResultNone:
            return "none";
        case telemetry::kSettingsResultApplied:
            return "applied";
        case telemetry::kSettingsResultRejected:
            return "rejected";
        case telemetry::kSettingsResultPersistFailed:
            return "persist failed";
        case telemetry::kSettingsResultStorageUnavailable:
            return "storage unavailable";
        default:
            return "unknown";
    }
}

bool SettingsStatusFlagSet(uint8_t statusFlags, uint8_t flag) {
    return (statusFlags & flag) != 0u;
}

telemetry::RuntimeSettingsPayloadV1 PayloadFromSnapshot(const telemetry::SettingsSnapshotV1 &snapshot) {
    return snapshot.payload;
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

struct WhisperModelChoice {
    const char *label;
    const char *fileName;
    const char *url;
};

constexpr std::array<WhisperModelChoice, 4> kWhisperModels = {{
    {"tiny.en (75 MB)", "ggml-tiny.en.bin", "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-tiny.en.bin"},
    {"base.en (142 MB)", "ggml-base.en.bin", "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin"},
    {"small.en (466 MB)", "ggml-small.en.bin", "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-small.en.bin"},
    {"medium.en (1.5 GB)", "ggml-medium.en.bin", "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-medium.en.bin"},
}};

std::string DefaultModelsDirectory(const char *argv0) {
    const char *envModelDir = std::getenv("WHISPER_MODEL_DIR");
    if (envModelDir != nullptr && envModelDir[0] != '\0') {
        return std::string(envModelDir);
    }

    std::vector<std::filesystem::path> candidates;
    candidates.emplace_back("models");

    if (argv0 != nullptr && argv0[0] != '\0') {
        const std::filesystem::path argPath(argv0);
        if (argPath.has_parent_path()) {
            candidates.push_back(argPath.parent_path() / "models");
        }
    }

    for (const std::filesystem::path &candidate : candidates) {
        if (std::filesystem::exists(candidate) && std::filesystem::is_directory(candidate)) {
            return candidate.string();
        }
    }

    return "models";
}

std::string ResolveWhisperModelPath(const char *argv0, const char *defaultFileName) {
    const char *envModel = std::getenv("WHISPER_MODEL_PATH");
    if (envModel != nullptr && envModel[0] != '\0' && std::filesystem::is_regular_file(envModel)) {
        return std::string(envModel);
    }

    const std::filesystem::path modelPath = std::filesystem::path(DefaultModelsDirectory(argv0)) / defaultFileName;
    return modelPath.string();
}

class WhisperModelDownloader {
  public:
    ~WhisperModelDownloader() { Stop(); }

    bool Start(const std::string &url, const std::string &outputPath) {
#if !defined(ACS_ENABLE_NATIVE_VOICE) || !ACS_ENABLE_NATIVE_VOICE
        (void)url;
        (void)outputPath;
        SetError("native voice support not built");
        return false;
#else
        if (running_.load()) {
            SetError("download already in progress");
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lastError_.clear();
            latestStatus_ = "starting download";
            completedPath_.clear();
            hasCompletedDownload_ = false;
            progress_ = 0.0f;
        }
        running_.store(true);
        worker_ = std::thread([this, url, outputPath]() { DownloadLoop(url, outputPath); });
        return true;
#endif
    }

    void Stop() {
        cancelRequested_.store(true);
        if (worker_.joinable()) {
            worker_.join();
        }
        cancelRequested_.store(false);
        running_.store(false);
    }

    bool Running() const { return running_.load(); }

    float Progress() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return progress_;
    }

    std::string Status() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return latestStatus_;
    }

    std::string LastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastError_;
    }

    bool ConsumeCompletedDownload(std::string *pathOut) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasCompletedDownload_) {
            return false;
        }
        *pathOut = completedPath_;
        hasCompletedDownload_ = false;
        return true;
    }

  private:
#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
    struct DownloadContext {
        WhisperModelDownloader *self = nullptr;
        FILE *file = nullptr;
    };

    static size_t WriteCallback(char *ptr, size_t size, size_t nmemb, void *userdata) {
        DownloadContext *context = static_cast<DownloadContext *>(userdata);
        return fwrite(ptr, size, nmemb, context->file);
    }

    static int ProgressCallback(void *clientp, curl_off_t total, curl_off_t now, curl_off_t, curl_off_t) {
        DownloadContext *context = static_cast<DownloadContext *>(clientp);
        if (context->self->cancelRequested_.load()) {
            return 1;
        }
        std::lock_guard<std::mutex> lock(context->self->mutex_);
        if (total > 0) {
            context->self->progress_ = static_cast<float>(static_cast<double>(now) / static_cast<double>(total));
        }
        char buffer[96];
        std::snprintf(buffer,
                      sizeof(buffer),
                      "downloading %.1f MB / %.1f MB",
                      static_cast<double>(now) / (1024.0 * 1024.0),
                      static_cast<double>(total) / (1024.0 * 1024.0));
        context->self->latestStatus_ = buffer;
        return 0;
    }

    void DownloadLoop(const std::string &url, const std::string &outputPath) {
        const std::filesystem::path outPath(outputPath);
        std::filesystem::create_directories(outPath.parent_path());
        const std::filesystem::path tempPath = outPath.string() + ".part";

        CURL *curl = curl_easy_init();
        if (curl == nullptr) {
            SetError("failed to initialize CURL");
            running_.store(false);
            return;
        }

        FILE *file = std::fopen(tempPath.string().c_str(), "wb");
        if (file == nullptr) {
            curl_easy_cleanup(curl);
            SetError("failed to open model output file");
            running_.store(false);
            return;
        }

        DownloadContext context{};
        context.self = this;
        context.file = file;

        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &context);
        curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, &ProgressCallback);
        curl_easy_setopt(curl, CURLOPT_XFERINFODATA, &context);
        curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0L);

        const CURLcode result = curl_easy_perform(curl);
        std::fclose(file);
        curl_easy_cleanup(curl);

        if (result != CURLE_OK) {
            std::filesystem::remove(tempPath);
            SetError(std::string("download failed: ") + curl_easy_strerror(result));
            running_.store(false);
            return;
        }

        std::error_code renameError;
        std::filesystem::rename(tempPath, outPath, renameError);
        if (renameError) {
            std::filesystem::remove(tempPath);
            SetError(std::string("failed to finalize model file: ") + renameError.message());
            running_.store(false);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            completedPath_ = outPath.string();
            hasCompletedDownload_ = true;
            progress_ = 1.0f;
            latestStatus_ = "download complete";
            lastError_.clear();
        }
        running_.store(false);
    }
#endif

    void SetError(const std::string &text) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastError_ = text;
        latestStatus_ = text;
    }

    mutable std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> cancelRequested_{false};
    std::thread worker_;
    float progress_ = 0.0f;
    std::string latestStatus_;
    std::string lastError_;
    std::string completedPath_;
    bool hasCompletedDownload_ = false;
};

class VoiceCommandReceiver {
  public:
    ~VoiceCommandReceiver() { Stop(); }

    bool Start(const char *modelPath) {
        if (running_.load()) {
            return true;
        }
#if !defined(ACS_ENABLE_NATIVE_VOICE) || !ACS_ENABLE_NATIVE_VOICE
        (void)modelPath;
        SetError("native voice support not built; provide whisper.cpp and PortAudio to CMake");
        return false;
#else
        if (modelPath == nullptr || modelPath[0] == '\0' || !std::filesystem::is_regular_file(modelPath)) {
            SetError("Whisper model not found; download one in the GUI or set WHISPER_MODEL_PATH");
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            lastError_.clear();
            latestTranscript_.clear();
            latestAcceptedCommand_.clear();
            hasPendingCommand_ = false;
            modelPath_ = modelPath;
        }
        {
            std::lock_guard<std::mutex> lock(audioMutex_);
            chunkQueue_.clear();
        }
        running_.store(true);
        captureWorker_ = std::thread([this]() { CaptureLoop(); });
        transcribeWorker_ = std::thread([this]() { TranscribeLoop(); });
        return true;
#endif
    }

    void Stop() {
        running_.store(false);
#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
        if (stream_ != nullptr) {
            Pa_AbortStream(stream_);
        }
#endif
        audioCv_.notify_all();

        if (captureWorker_.joinable()) {
            captureWorker_.join();
        }
        if (transcribeWorker_.joinable()) {
            transcribeWorker_.join();
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
    void PublishTranscriptLocked(const std::string &line) {
        latestTranscript_ = line;
        const std::optional<ParsedVoiceCommand> parsed = ParseVoiceCommand(line);
        if (!parsed.has_value()) {
            return;
        }
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

    void SetError(const std::string &text) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastError_ = text;
    }

    void CaptureLoop() {
#if !defined(ACS_ENABLE_NATIVE_VOICE) || !ACS_ENABLE_NATIVE_VOICE
        return;
#else
        if (Pa_Initialize() != paNoError) {
            SetError("PortAudio initialization failed");
            running_.store(false);
            audioCv_.notify_all();
            return;
        }

        const PaError openError =
            Pa_OpenDefaultStream(&stream_, 1, 0, paInt16, kSampleRate, kFramesPerBuffer, nullptr, nullptr);
        if (openError != paNoError) {
            SetError(std::string("PortAudio open failed: ") + Pa_GetErrorText(openError));
            Pa_Terminate();
            running_.store(false);
            audioCv_.notify_all();
            return;
        }

        const PaError startError = Pa_StartStream(stream_);
        if (startError != paNoError) {
            SetError(std::string("PortAudio start failed: ") + Pa_GetErrorText(startError));
            Pa_CloseStream(stream_);
            stream_ = nullptr;
            Pa_Terminate();
            running_.store(false);
            audioCv_.notify_all();
            return;
        }

        std::vector<int16_t> audioBuffer(kFramesPerBuffer);
        std::vector<float> captureBuffer;
        captureBuffer.reserve(kChunkSamples * 2);
        while (running_.load()) {
            const PaError readError = Pa_ReadStream(stream_, audioBuffer.data(), kFramesPerBuffer);
            if (readError == paInputOverflowed) {
                continue;
            }
            if (readError != paNoError) {
                if (running_.load()) {
                    SetError(std::string("PortAudio read failed: ") + Pa_GetErrorText(readError));
                }
                break;
            }

            for (int16_t sample : audioBuffer) {
                captureBuffer.push_back(static_cast<float>(sample) / 32768.0f);
            }

            while (captureBuffer.size() >= static_cast<size_t>(kChunkSamples)) {
                std::vector<float> chunk(captureBuffer.begin(), captureBuffer.begin() + kChunkSamples);
                captureBuffer.erase(captureBuffer.begin(), captureBuffer.begin() + kChunkSamples);

                double sumSquares = 0.0;
                for (float sample : chunk) {
                    sumSquares += static_cast<double>(sample) * static_cast<double>(sample);
                }
                const float rms = std::sqrt(static_cast<float>(sumSquares / std::max<size_t>(1, chunk.size())));
                if (rms < kMinRms) {
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(audioMutex_);
                    while (chunkQueue_.size() >= kMaxQueuedChunks) {
                        chunkQueue_.pop_front();
                    }
                    chunkQueue_.push_back(std::move(chunk));
                }
                audioCv_.notify_one();
            }
        }

        if (stream_ != nullptr) {
            Pa_StopStream(stream_);
            Pa_CloseStream(stream_);
            stream_ = nullptr;
        }
        running_.store(false);
        Pa_Terminate();
        audioCv_.notify_all();
#endif
    }

    void TranscribeLoop() {
#if !defined(ACS_ENABLE_NATIVE_VOICE) || !ACS_ENABLE_NATIVE_VOICE
        return;
#else
        whisper_context_params contextParams = whisper_context_default_params();
        whisper_context *context = whisper_init_from_file_with_params(modelPath_.c_str(), contextParams);
        if (context == nullptr) {
            SetError("failed to load Whisper model");
            running_.store(false);
            audioCv_.notify_all();
            if (stream_ != nullptr) {
                Pa_AbortStream(stream_);
            }
            return;
        }

        std::string lastPublishedTranscript;
        while (running_.load()) {
            std::vector<float> chunk;
            {
                std::unique_lock<std::mutex> lock(audioMutex_);
                audioCv_.wait(lock, [this]() { return !running_.load() || !chunkQueue_.empty(); });
                if (!running_.load() && chunkQueue_.empty()) {
                    break;
                }
                chunk = std::move(chunkQueue_.front());
                chunkQueue_.pop_front();
            }

            whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
            params.print_progress = false;
            params.print_realtime = false;
            params.print_timestamps = false;
            params.print_special = false;
            params.translate = false;
            params.language = "en";
            params.n_threads = std::max(1u, std::thread::hardware_concurrency() / 2u);
            const int whisperResult = whisper_full(context, params, chunk.data(), static_cast<int>(chunk.size()));
            if (whisperResult != 0) {
                continue;
            }

            std::string transcript;
            const int segmentCount = whisper_full_n_segments(context);
            for (int i = 0; i < segmentCount; ++i) {
                transcript += whisper_full_get_segment_text(context, i);
            }
            transcript = TrimString(transcript);
            if (!transcript.empty() && ToLowerString(transcript) != lastPublishedTranscript) {
                lastPublishedTranscript = ToLowerString(transcript);
                std::lock_guard<std::mutex> lock(mutex_);
                PublishTranscriptLocked(transcript);
            }
        }

        whisper_free(context);
#endif
    }

    static constexpr float kSampleRate = 16000.0f;
    static constexpr unsigned long kFramesPerBuffer = 1024;
    static constexpr size_t kChunkSamples = 16000 * 3;
    static constexpr float kMinRms = 0.008f;
    static constexpr size_t kMaxQueuedChunks = 2;

    mutable std::mutex mutex_;
    std::mutex audioMutex_;
    std::condition_variable audioCv_;
    std::atomic<bool> running_{false};
    std::thread captureWorker_;
    std::thread transcribeWorker_;
    std::deque<std::vector<float>> chunkQueue_;
    std::string lastError_;
    std::string latestTranscript_;
    std::string latestAcceptedCommand_;
    ParsedVoiceCommand pendingCommand_{};
    bool hasPendingCommand_ = false;
    std::string modelPath_;
#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
    PaStream *stream_ = nullptr;
#endif
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
            commandBurstPacketsRemaining_ = kCommandBurstPacketCount;
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
        commandBurstPacketsRemaining_ = kCommandBurstPacketCount;
        disconnectPacketsRemaining_ = 5;
    }

    std::string LastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastError_;
    }

    bool HeartbeatEnabled() const { return heartbeatEnabled_; }

    uint32_t RequestSettingsSnapshot() {
        telemetry::RuntimeSettingsPayloadV1 payload{};
        return QueueSettingsCommand(telemetry::kSettingsOpRequestCurrent, payload);
    }

    uint32_t ApplyAndPersistSettings(const telemetry::RuntimeSettingsPayloadV1 &payload) {
        return QueueSettingsCommand(telemetry::kSettingsOpApplyAndPersist, payload);
    }

    uint32_t RestoreDefaultSettings() {
        telemetry::RuntimeSettingsPayloadV1 payload{};
        return QueueSettingsCommand(telemetry::kSettingsOpRestoreDefaults, payload);
    }

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
            bool sendBurstPacket = false;
            {
                std::lock_guard<std::mutex> lock(commandMutex_);
                sendBurstPacket = commandBurstPacketsRemaining_ > 0;
            }
            const auto periodicActuationInterval = std::chrono::milliseconds(100);
            const auto burstActuationInterval = std::chrono::milliseconds(25);
            const auto requiredInterval = sendBurstPacket ? burstActuationInterval : periodicActuationInterval;
            if (heartbeatEnabled_ &&
                (sendActuationNow || (sendBurstPacket && (now - lastActuationCommandSent_) >= requiredInterval) ||
                 (now - lastActuationCommandSent_) >= periodicActuationInterval)) {
                telemetry::ActuationCommandV1 command{};
                {
                    std::lock_guard<std::mutex> lock(commandMutex_);
                    command.mode = manualActuationOverride_ ? telemetry::kActuationModeManual
                                                            : telemetry::kActuationModeAuto;
                    command.angleDeg = manualActuationAngleDeg_;
                    if (commandBurstPacketsRemaining_ > 0) {
                        --commandBurstPacketsRemaining_;
                    }
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
            bool sendSettingsPacket = false;
            {
                std::lock_guard<std::mutex> lock(commandMutex_);
                sendSettingsPacket = settingsBurstPacketsRemaining_ > 0;
            }
            if (heartbeatEnabled_ && sendSettingsPacket &&
                (now - lastSettingsCommandSent_) >= std::chrono::milliseconds(75)) {
                telemetry::SettingsCommandV1 command{};
                {
                    std::lock_guard<std::mutex> lock(commandMutex_);
                    command = pendingSettingsCommand_;
                    if (settingsBurstPacketsRemaining_ > 0) {
                        --settingsBurstPacketsRemaining_;
                    }
                }
                sendto(socketFd_,
                       &command,
                       sizeof(command),
                       0,
                       reinterpret_cast<const sockaddr *>(&teensyAddr_),
                       sizeof(teensyAddr_));
                lastSettingsCommandSent_ = now;
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

            std::array<uint8_t, 256> packetBuffer{};
            sockaddr_in src{};
            socklen_t srcLen = sizeof(src);
            const ssize_t n = recvfrom(socketFd_,
                                       packetBuffer.data(),
                                       packetBuffer.size(),
                                       0,
                                       reinterpret_cast<sockaddr *>(&src),
                                       &srcLen);
            if (n < 8) {
                continue;
            }

            uint32_t magic = 0;
            std::memcpy(&magic, packetBuffer.data(), sizeof(magic));
            if (magic == telemetry::kPacketMagic && n == static_cast<ssize_t>(sizeof(telemetry::PacketV1))) {
                telemetry::PacketV1 packet{};
                std::memcpy(&packet, packetBuffer.data(), sizeof(packet));
                if (packet.version != telemetry::kPacketVersion || packet.size != sizeof(telemetry::PacketV1)) {
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
                continue;
            }

            if (magic == telemetry::kSettingsSnapshotMagic &&
                n == static_cast<ssize_t>(sizeof(telemetry::SettingsSnapshotV1))) {
                telemetry::SettingsSnapshotV1 settings{};
                std::memcpy(&settings, packetBuffer.data(), sizeof(settings));
                if (settings.version != telemetry::kSettingsSnapshotVersion ||
                    settings.size != sizeof(telemetry::SettingsSnapshotV1)) {
                    continue;
                }

                std::lock_guard<std::mutex> lock(mutex_);
                shared_.settings = settings;
                shared_.hasSettings = true;
                shared_.lastSettingsRx = std::chrono::steady_clock::now();
                continue;
            }
        }
    }

    uint32_t QueueSettingsCommand(uint8_t operation, const telemetry::RuntimeSettingsPayloadV1 &payload) {
        std::lock_guard<std::mutex> lock(commandMutex_);
        telemetry::SettingsCommandV1 command{};
        command.operation = operation;
        command.requestId = nextSettingsRequestId_++;
        command.payload = payload;
        pendingSettingsCommand_ = command;
        settingsBurstPacketsRemaining_ = kSettingsBurstPacketCount;
        return command.requestId;
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
    std::chrono::steady_clock::time_point lastSettingsCommandSent_ = std::chrono::steady_clock::time_point::min();
    mutable std::mutex commandMutex_;
    bool manualActuationOverride_ = false;
    float manualActuationAngleDeg_ = 0.0f;
    bool sendActuationImmediately_ = false;
    bool telemetryStreamingEnabled_ = true;
    uint8_t disconnectPacketsRemaining_ = 0;
    static constexpr uint8_t kCommandBurstPacketCount = 8;
    uint8_t commandBurstPacketsRemaining_ = 0;
    static constexpr uint8_t kSettingsBurstPacketCount = 4;
    uint8_t settingsBurstPacketsRemaining_ = 0;
    telemetry::SettingsCommandV1 pendingSettingsCommand_{};
    uint32_t nextSettingsRequestId_ = 1;
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

#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
    curl_global_init(CURL_GLOBAL_DEFAULT);
#endif

    UdpReceiver receiver(port, teensyIp, teensyPort);
    receiver.Start();
    uint32_t pendingSettingsRequestId = receiver.RequestSettingsSnapshot();
    int selectedWhisperModelIndex = 1;
    std::string voiceModelPath = ResolveWhisperModelPath(argv[0], kWhisperModels[selectedWhisperModelIndex].fileName);
    bool manualActuationOverride = false;
    float manualActuationAngleDeg = 0.0f;
    bool telemetryStreamingEnabled = true;
    bool voiceListenEnabled = false;
    VoiceCommandReceiver voiceReceiver;
    WhisperModelDownloader modelDownloader;
    DashboardState dashboard{};
    telemetry::RuntimeSettingsPayloadV1 settingsDraft{};
    bool settingsDraftInitialized = false;
    bool settingsDraftDirty = false;
    uint32_t lastSettingsRevisionSeen = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        const SharedTelemetry snap = receiver.Snapshot();
        std::string completedModelPath;
        if (modelDownloader.ConsumeCompletedDownload(&completedModelPath)) {
            voiceModelPath = completedModelPath;
        }
        if (snap.hasSettings) {
            const bool shouldRefreshDraft =
                !settingsDraftInitialized ||
                (!settingsDraftDirty && snap.settings.settingsRevision != lastSettingsRevisionSeen) ||
                (pendingSettingsRequestId != 0 && snap.settings.appliedRequestId == pendingSettingsRequestId);
            if (shouldRefreshDraft) {
                settingsDraft = PayloadFromSnapshot(snap.settings);
                settingsDraftInitialized = true;
                settingsDraftDirty = false;
                lastSettingsRevisionSeen = snap.settings.settingsRevision;
            }
            if (pendingSettingsRequestId != 0 && snap.settings.appliedRequestId == pendingSettingsRequestId) {
                pendingSettingsRequestId = 0;
            }
        }
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
        if (ImGui::BeginTabBar("main_tabs")) {
            if (ImGui::BeginTabItem("Flight")) {
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
                    const WhisperModelChoice &selectedModel = kWhisperModels[selectedWhisperModelIndex];
                    const std::filesystem::path selectedModelPath =
                        std::filesystem::path(DefaultModelsDirectory(argv[0])) / selectedModel.fileName;
                    if (ImGui::BeginCombo("Whisper model", selectedModel.label)) {
                        for (int i = 0; i < static_cast<int>(kWhisperModels.size()); ++i) {
                            const bool isSelected = (selectedWhisperModelIndex == i);
                            if (ImGui::Selectable(kWhisperModels[i].label, isSelected)) {
                                selectedWhisperModelIndex = i;
                                if (!voiceListenEnabled) {
                                    voiceModelPath = ResolveWhisperModelPath(argv[0], kWhisperModels[i].fileName);
                                }
                            }
                            if (isSelected) {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }
                    if (ImGui::Button("Use selected model")) {
                        voiceModelPath = selectedModelPath.string();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Download selected model")) {
                        modelDownloader.Start(selectedModel.url, selectedModelPath.string());
                    }
                    ImGui::TextDisabled("Active model: %s", voiceModelPath.c_str());
                    ImGui::TextDisabled("Selected model file: %s", selectedModelPath.string().c_str());
                    if (std::filesystem::exists(selectedModelPath)) {
                        ImGui::TextColored(ImVec4(0.46f, 0.86f, 0.58f, 1.0f), "Selected model is present on disk");
                    } else {
                        ImGui::TextColored(ImVec4(0.98f, 0.68f, 0.26f, 1.0f), "Selected model has not been downloaded yet");
                    }
                    if (modelDownloader.Running()) {
                        ImGui::ProgressBar(modelDownloader.Progress(), ImVec2(-1.0f, 8.0f), "");
                        ImGui::TextDisabled("%s", modelDownloader.Status().c_str());
                    } else {
                        const std::string downloadStatus = modelDownloader.Status();
                        if (!downloadStatus.empty()) {
                            ImGui::TextDisabled("%s", downloadStatus.c_str());
                        }
                    }
                    const std::string downloadError = modelDownloader.LastError();
                    if (!downloadError.empty()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s", downloadError.c_str());
                    }

                    ImGui::Separator();
                    bool voiceToggle = voiceListenEnabled;
                    if (ImGui::Checkbox("Voice listen (wake: ACS/Apogee)", &voiceToggle)) {
                        if (voiceToggle) {
                            voiceListenEnabled = voiceReceiver.Start(voiceModelPath.c_str());
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
                        ImGui::TextDisabled("Whisper commands (wake word optional):");
                        ImGui::BulletText("\"move 30\", \"set angle 45\", \"actuate sixty\"");
                        ImGui::BulletText("\"thirty\", \"three zero\", \"full\", \"half\"");
                        ImGui::BulletText("\"return to auto\", \"auto off\", \"disable automatic\"");
                        ImGui::TextDisabled("Model path: %s", voiceModelPath.c_str());
                    }
                    const std::string transcript = voiceReceiver.LatestTranscript();
                    if (!transcript.empty()) {
                        ImGui::TextWrapped("Heard: %s", transcript.c_str());
                    }
                    const std::string accepted = voiceReceiver.LatestAcceptedCommand();
                    if (!accepted.empty()) {
                        ImGui::TextColored(ImVec4(0.50f, 0.90f, 0.62f, 1.0f), "%s", accepted.c_str());
                    }

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
                        if (manualActuationOverride != flightManualOverride) {
                            ImGui::TextColored(ImVec4(0.98f, 0.68f, 0.26f, 1.0f),
                                               "Manual command sent; waiting for firmware acknowledgement");
                        } else {
                            ImGui::TextColored(ImVec4(0.46f, 0.86f, 0.58f, 1.0f),
                                               "Ground command matches firmware mode");
                        }
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
                            ImGui::Text("Main quaternion source: %s",
                                        MainQuaternionSourceName(p.sensorMainQuaternionSource));
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

                        if (ImGui::CollapsingHeader("Control Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) {
                            ImGui::Text("Auto command: %.2f deg", p.sensorAutoCommandDeg);
                            ImGui::Text("Actuation settling: %s", p.sensorActuationIsSettling > 0.5f ? "true" : "false");
                            ImGui::Text("Best predicted apogee: %.2f m", p.sensorOptimizerBestPredictedApogeeM);
                            ImGui::Text("Optimizer cost: %.3f", p.sensorOptimizerBestCost);
                            ImGui::Text("Time to apogee: %.2f s", p.sensorOptimizerTimeToApogeeS);
                            ImGui::Text("Baro sigma scale / gate: %.2f / %.2f",
                                        p.sensorAltimeterSigmaScale,
                                        p.sensorAltimeterGateSigma);
                            ImGui::Text("Predictor horiz speed: %.2f m/s", p.sensorPredictorSeedHorizontalSpeedMps);
                            ImGui::Text("Predictor zenith: %.3f rad", p.sensorPredictorSeedClampedZenithRad);
                            ImGui::Text("Predictor ang rate: %.3f rad/s",
                                        p.sensorPredictorSeedClampedAngularRateRadPerSec);
                            ImGui::Text("Predictor flags: %.0f", p.sensorPredictorSeedConfidenceFlags);
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
                ImGui::EndTabItem();
            }

            if (ImGui::BeginTabItem("Settings")) {
                if (!snap.hasSettings) {
                    ImGui::TextDisabled("Waiting for runtime settings snapshot from Teensy.");
                    if (ImGui::Button("Request Settings")) {
                        pendingSettingsRequestId = receiver.RequestSettingsSnapshot();
                    }
                } else {
                    const telemetry::SettingsSnapshotV1 &settings = snap.settings;
                    ImGui::Text("Runtime Settings");
                    ImGui::TextDisabled("Apply/save is intended for preflight use while the flight computer is on the ground.");
                    ImGui::Text("Revision: %u", settings.settingsRevision);
                    ImGui::Text("Last request id: %u", settings.appliedRequestId);
                    ImGui::Text("Last result: %s", SettingsResultName(settings.lastCommandResult));
                    if (pendingSettingsRequestId != 0) {
                        ImGui::TextColored(ImVec4(0.98f, 0.68f, 0.26f, 1.0f),
                                           "Awaiting Teensy acknowledgement for request %u",
                                           pendingSettingsRequestId);
                    }
                    ImGui::Separator();
                    ImGui::Text("Storage");
                    ImGui::BulletText("SD available: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusStorageAvailable)
                                          ? "yes"
                                          : "no");
                    ImGui::BulletText("Settings file present: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusFilePresent)
                                          ? "yes"
                                          : "no");
                    ImGui::BulletText("Using defaults: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusUsingDefaults)
                                          ? "yes"
                                          : "no");
                    ImGui::BulletText("Last load ok: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusLastLoadSucceeded)
                                          ? "yes"
                                          : "no");
                    ImGui::BulletText("Last save ok: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusLastSaveSucceeded)
                                          ? "yes"
                                          : "no");
                    ImGui::BulletText("Default file created: %s",
                                      SettingsStatusFlagSet(settings.statusFlags, telemetry::kSettingsStatusCreatedDefaultFile)
                                          ? "yes"
                                          : "no");
                    ImGui::Separator();

                    if (ImGui::Button("Refresh From Teensy")) {
                        pendingSettingsRequestId = receiver.RequestSettingsSnapshot();
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Revert Local Edits")) {
                        settingsDraft = PayloadFromSnapshot(settings);
                        settingsDraftDirty = false;
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Restore Defaults On Teensy")) {
                        pendingSettingsRequestId = receiver.RestoreDefaultSettings();
                    }

                    if (!settingsDraftInitialized) {
                        settingsDraft = PayloadFromSnapshot(settings);
                        settingsDraftInitialized = true;
                    }

                    ImGui::BeginDisabled(!settingsDraftInitialized);
                    if (ImGui::Button("Apply And Save To Teensy")) {
                        pendingSettingsRequestId = receiver.ApplyAndPersistSettings(settingsDraft);
                    }
                    ImGui::EndDisabled();
                    ImGui::TextDisabled("%s", settingsDraftDirty ? "local edits not yet saved" : "local draft matches latest snapshot");
                    ImGui::Separator();

                    ImGui::Text("Environment");
                    settingsDraftDirty |= ImGui::InputDouble("Ground temperature (F)", &settingsDraft.groundTemperatureF, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Wind speed (mph)", &settingsDraft.windSpeedMph, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Wind direction (deg)", &settingsDraft.windDirectionDeg, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Launch direction (deg)", &settingsDraft.launchDirectionDeg, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Roughness length (m)", &settingsDraft.roughnessLengthMeters, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Gradient height (m)", &settingsDraft.gradientHeightMeters, 0.0, 0.0, "%.6f");
                    settingsDraftDirty |= ImGui::InputDouble("Measurement height (m)", &settingsDraft.measurementHeightMeters, 0.0, 0.0, "%.6f");

                    ImGui::Separator();
                    ImGui::Text("Vehicle");
                    settingsDraftDirty |= ImGui::InputDouble("CP offset (m)", &settingsDraft.centerOfPressureOffsetMeters, 0.0, 0.0, "%.8f");
                    settingsDraftDirty |= ImGui::InputDouble("Moment of inertia (kg*m^2)", &settingsDraft.momentOfInertiaKgM2, 0.0, 0.0, "%.8f");
                    settingsDraftDirty |= ImGui::InputDouble("Dry mass (kg)", &settingsDraft.dryMassKg, 0.0, 0.0, "%.8f");
                }
                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        receiver.SetActuationOverride(manualActuationOverride, manualActuationAngleDeg);
        receiver.SetTelemetryStreamingEnabled(telemetryStreamingEnabled);

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
    modelDownloader.Stop();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
#if defined(ACS_ENABLE_NATIVE_VOICE) && ACS_ENABLE_NATIVE_VOICE
    curl_global_cleanup();
#endif
    return 0;
}
