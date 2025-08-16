#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <chrono>
#include <thread>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdio>
#include <cerrno>

struct IntanDataHeader {
    uint32_t magic;        // "INTA" 0x494E5441
    uint32_t timestamp;    // increment per frame
    uint32_t dataSize;     // total bytes in shared region used
    uint32_t streamCount;  // streams
    uint32_t channelCount; // channels per stream
    uint32_t sampleRate;   // Hz
};

struct IntanDataBlock {
    uint32_t streamId;
    uint32_t channelId;
    float value;           // microvolts
};

int main() {
    const char* shmName = "/intan_rhx_shm_v1";
    const int SAMPLE_RATE = 30000;
    const int NUM_STREAMS = 4;
    const int NUM_CHANNELS = 32;
    const int SAMPLES_PER_FRAME = 128; // align with RHX block

    size_t blocks = static_cast<size_t>(NUM_STREAMS) * NUM_CHANNELS * SAMPLES_PER_FRAME;
    size_t shmSize = sizeof(IntanDataHeader) + blocks * sizeof(IntanDataBlock);

    // Remove any stale segment from previous runs to avoid size mismatch
    shm_unlink(shmName);
    int fd = shm_open(shmName, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        std::perror("shm_open failed");
        return 1;
    }
    if (ftruncate(fd, shmSize) != 0) {
        // If another process already created with the correct size, we can proceed
        std::perror("ftruncate failed");
        struct stat st{};
        if (fstat(fd, &st) == 0 && static_cast<size_t>(st.st_size) == shmSize) {
            // ok, already expected size
        } else {
            std::cerr << "Shared memory size mismatch. Start supplier before the app, or close the app and retry." << std::endl;
            close(fd);
            return 1;
        }
    }
    void* base = mmap(nullptr, shmSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        std::cerr << "mmap failed" << std::endl;
        return 1;
    }

    auto* header = reinterpret_cast<IntanDataHeader*>(base);
    auto* data = reinterpret_cast<IntanDataBlock*>(reinterpret_cast<uint8_t*>(base) + sizeof(IntanDataHeader));

    // Synthetic Neural-like source per channel: LFP + spikes + noise (approximate SynthDataBlockGenerator)
    const double dtMs = 1000.0 / SAMPLE_RATE;
    const double LFPFreqHz = 2.3;
    const double LFPModHz = 0.5;
    const double NoiseRMS = 5.0; // uV
    const double SpikeRefractoryMs = 5.0;
    // Per-channel state
    std::vector<std::vector<double>> tMs(NUM_STREAMS, std::vector<double>(NUM_CHANNELS, 0.0));
    std::vector<std::vector<bool>> firing(NUM_STREAMS, std::vector<bool>(NUM_CHANNELS, false));
    std::vector<std::vector<double>> spikeTimeMs(NUM_STREAMS, std::vector<double>(NUM_CHANNELS, 0.0));
    std::vector<std::vector<double>> spikeAmp(NUM_STREAMS, std::vector<double>(NUM_CHANNELS, -300.0));
    std::vector<std::vector<double>> spikeDurMs(NUM_STREAMS, std::vector<double>(NUM_CHANNELS, 1.0));
    std::vector<std::vector<double>> spikeRateHz(NUM_STREAMS, std::vector<double>(NUM_CHANNELS, 5.0));
    // Initialize per-channel params (deterministic pseudo-random)
    for (int s = 0; s < NUM_STREAMS; ++s) {
        for (int c = 0; c < NUM_CHANNELS; ++c) {
            spikeAmp[s][c] = -200.0 - 300.0 * (static_cast<double>((s*37 + c*73) % 100) / 100.0);
            spikeDurMs[s][c] = 0.3 + 1.4 * (static_cast<double>((s*57 + c*11) % 100) / 100.0);
            double u = static_cast<double>((s*91 + c*29) % 100) / 100.0;
            // log-uniform 0.1..50 Hz
            spikeRateHz[s][c] = std::pow(10.0, std::log10(0.1) + (std::log10(50.0) - std::log10(0.1)) * u);
        }
    }

    uint32_t timestamp = 0;
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        header->magic = 0x494E5441;
        header->timestamp = timestamp;
        header->streamCount = NUM_STREAMS;
        header->channelCount = NUM_CHANNELS;
        header->sampleRate = SAMPLE_RATE;
        header->dataSize = static_cast<uint32_t>(shmSize);

        size_t w = 0;
        for (int n = 0; n < SAMPLES_PER_FRAME; ++n) {
            for (int s = 0; s < NUM_STREAMS; ++s) {
                for (int c = 0; c < NUM_CHANNELS; ++c) {
                    // LFP component with slow modulation
                    double lfpAmp = 100.0 + 80.0 * std::sin(2.0 * M_PI * LFPModHz * (tMs[s][c] / 1000.0));
                    double lfp = lfpAmp * std::sin(2.0 * M_PI * LFPFreqHz * (tMs[s][c] / 1000.0));
                    // Spikes
                    double spike = 0.0;
                    if (firing[s][c]) {
                        if (spikeTimeMs[s][c] < spikeDurMs[s][c]) {
                            spike = spikeAmp[s][c] * std::exp(-2.0 * spikeTimeMs[s][c]) *
                                    std::sin(2.0 * M_PI * (spikeTimeMs[s][c] / spikeDurMs[s][c]));
                            spikeTimeMs[s][c] += dtMs;
                        } else if (spikeTimeMs[s][c] < spikeDurMs[s][c] + SpikeRefractoryMs) {
                            spikeTimeMs[s][c] += dtMs;
                        } else {
                            firing[s][c] = false;
                            spikeTimeMs[s][c] = 0.0;
                        }
                    } else {
                        double spikeMod = (1000.0 - std::fmod(tMs[s][c], 1000.0)) / 1000.0;
                        double p = spikeMod * spikeRateHz[s][c] * (dtMs / 1000.0);
                        double r = static_cast<double>(std::rand()) / RAND_MAX;
                        if (r < p) {
                            firing[s][c] = true;
                        }
                    }
                    // Noise (approx Gaussian via sum of uniforms)
                    double noise = 0.0;
                    for (int k = 0; k < 3; ++k) noise += (static_cast<double>(std::rand()) / RAND_MAX - 0.5);
                    noise *= (NoiseRMS / 0.612372); // scale 3-sum to ~1 rms

                    float value = static_cast<float>(lfp + spike + noise);
                    data[w++] = IntanDataBlock{static_cast<uint32_t>(s), static_cast<uint32_t>(c), value};
                    tMs[s][c] += dtMs;
                }
            }
        }

        // pacing to maintain sample rate
        timestamp += SAMPLES_PER_FRAME;
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start);
        auto target = std::chrono::microseconds(timestamp * 1000000LL / SAMPLE_RATE);
        if (elapsed < target) std::this_thread::sleep_for(target - elapsed);
    }

    munmap(base, shmSize);
    close(fd);
    return 0;
}


