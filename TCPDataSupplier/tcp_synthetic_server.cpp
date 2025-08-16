#include <iostream>
#include <vector>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>

// Intan data structures for TCP communication
struct IntanDataHeader {
    uint32_t magic;        // Magic number "INTA" (0x494E5441)
    uint32_t timestamp;    // Timestamp
    uint32_t dataSize;     // Total size
    uint32_t streamCount;  // Number of streams
    uint32_t channelCount; // Number of channels
    uint32_t sampleRate;   // Sample rate
};

struct IntanDataBlock {
    uint32_t streamId;
    uint32_t channelId;
    float value;
};

class TCPSyntheticDataServer {
private:
    int serverSocket;
    int clientSocket;
    bool running;
    const int PORT = 12345;
    const int SAMPLE_RATE = 30000; // 30kHz
    const int NUM_STREAMS = 4;
    const int NUM_CHANNELS = 32;
    const int SAMPLES_PER_PACKET = 100;
    
    // Synthetic data generation parameters (copied from SynthDataBlockGenerator)
    std::vector<std::vector<double>> synthSources;
    std::vector<std::vector<int>> synthSampleIndex;
    
public:
    TCPSyntheticDataServer() : running(false) {
        // Initialize synthetic data sources (same as SynthDataBlockGenerator)
        synthSources.resize(NUM_STREAMS);
        synthSampleIndex.resize(NUM_STREAMS);
        
        for (int stream = 0; stream < NUM_STREAMS; ++stream) {
            synthSources[stream].resize(NUM_CHANNELS);
            synthSampleIndex[stream].resize(NUM_CHANNELS, 0);
            
            for (int channel = 0; channel < NUM_CHANNELS; ++channel) {
                // Initialize with different frequencies for each channel
                double baseFreq = 10.0 + (channel % 8) * 5.0; // 10Hz to 45Hz
                synthSources[stream][channel] = baseFreq;
            }
        }
    }
    
    ~TCPSyntheticDataServer() {
        stop();
    }
    
    bool start() {
        serverSocket = socket(AF_INET, SOCK_STREAM, 0);
        if (serverSocket < 0) {
            std::cerr << "Failed to create socket" << std::endl;
            return false;
        }
        
        int opt = 1;
        setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in serverAddr;
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_addr.s_addr = INADDR_ANY;
        serverAddr.sin_port = htons(PORT);
        
        if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
            std::cerr << "Failed to bind socket" << std::endl;
            close(serverSocket);
            return false;
        }
        
        if (listen(serverSocket, 1) < 0) {
            std::cerr << "Failed to listen" << std::endl;
            close(serverSocket);
            return false;
        }
        
        std::cout << "TCP Synthetic Data Server listening on port " << PORT << std::endl;
        std::cout << "Waiting for Intan RHX client connection..." << std::endl;
        
        // Accept client connection
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
        
        if (clientSocket < 0) {
            std::cerr << "Failed to accept connection" << std::endl;
            close(serverSocket);
            return false;
        }
        
        std::cout << "Client connected from " << inet_ntoa(clientAddr.sin_addr) << std::endl;
        running = true;
        
        return true;
    }
    
    void stop() {
        running = false;
        if (clientSocket >= 0) {
            close(clientSocket);
            clientSocket = -1;
        }
        if (serverSocket >= 0) {
            close(serverSocket);
            serverSocket = -1;
        }
    }
    
    // Generate synthetic data sample (same logic as SynthDataBlockGenerator)
    double generateSample(int stream, int channel) {
        double freq = synthSources[stream][channel];
        double time = static_cast<double>(synthSampleIndex[stream][channel]) / SAMPLE_RATE;
        
        // Generate sine wave with some noise
        double signal = sin(2.0 * M_PI * freq * time) * 100.0; // 100 microvolts amplitude
        double noise = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 20.0; // 20 microvolts noise
        
        synthSampleIndex[stream][channel]++;
        return signal + noise;
    }
    
    void run() {
        if (!running) return;
        
        std::cout << "Starting synthetic data transmission at " << SAMPLE_RATE << " Hz" << std::endl;
        std::cout << "Sending " << SAMPLES_PER_PACKET << " samples per packet" << std::endl;
        
        auto startTime = std::chrono::high_resolution_clock::now();
        uint32_t timestamp = 0;
        
        while (running) {
            // Generate data packet
            std::vector<uint8_t> packet;
            
            // Create header
            IntanDataHeader header;
            header.magic = 0x494E5441; // "INTA"
            header.timestamp = timestamp;
            header.streamCount = NUM_STREAMS;
            header.channelCount = NUM_CHANNELS;
            header.sampleRate = SAMPLE_RATE;
            
            // Calculate data size
            size_t dataBlockSize = NUM_STREAMS * NUM_CHANNELS * SAMPLES_PER_PACKET * sizeof(IntanDataBlock);
            header.dataSize = sizeof(IntanDataHeader) + dataBlockSize;
            
            // Add header to packet
            packet.insert(packet.end(), (uint8_t*)&header, (uint8_t*)&header + sizeof(IntanDataHeader));
            
            // Generate data blocks for all streams, channels, and samples
            for (int sample = 0; sample < SAMPLES_PER_PACKET; ++sample) {
                for (int stream = 0; stream < NUM_STREAMS; ++stream) {
                    for (int channel = 0; channel < NUM_CHANNELS; ++channel) {
                        IntanDataBlock block;
                        block.streamId = stream;
                        block.channelId = channel;
                        block.value = generateSample(stream, channel);
                        
                        packet.insert(packet.end(), (uint8_t*)&block, (uint8_t*)&block + sizeof(IntanDataBlock));
                    }
                }
            }
            
            // Send packet
            if (send(clientSocket, packet.data(), packet.size(), 0) < 0) {
                std::cerr << "Failed to send data packet" << std::endl;
                break;
            }
            
            std::cout << "Sent packet: timestamp=" << timestamp 
                      << " streams=" << NUM_STREAMS 
                      << " channels=" << NUM_CHANNELS 
                      << " samples=" << SAMPLES_PER_PACKET 
                      << " size=" << packet.size() << " bytes" << std::endl;
            
            timestamp += SAMPLES_PER_PACKET;
            
            // Sleep to maintain sample rate
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
            auto targetTime = std::chrono::microseconds(timestamp * 1000000 / SAMPLE_RATE);
            
            if (elapsed < targetTime) {
                std::this_thread::sleep_for(targetTime - elapsed);
            }
        }
        
        std::cout << "Data transmission stopped" << std::endl;
    }
};

int main() {
    std::cout << "=== Intan RHX TCP Synthetic Data Server ===" << std::endl;
    std::cout << "This server generates the same synthetic data as SynthDataBlockGenerator" << std::endl;
    std::cout << "and sends it over TCP for the PipelineDataRHXController to receive." << std::endl;
    std::cout << "===========================================" << std::endl;
    
    TCPSyntheticDataServer server;
    
    if (!server.start()) {
        std::cerr << "Failed to start server" << std::endl;
        return 1;
    }
    
    // Run in separate thread to allow for graceful shutdown
    std::thread dataThread([&server]() {
        server.run();
    });
    
    std::cout << "Press Enter to stop the server..." << std::endl;
    std::cin.get();
    
    server.stop();
    dataThread.join();
    
    std::cout << "Server stopped" << std::endl;
    return 0;
}
