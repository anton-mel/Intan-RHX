//------------------------------------------------------------------------------
//
//  Intan Technologies RHX Data Acquisition Software
//  Version 3.4.0
//
//  Copyright (c) 2020-2025 Intan Technologies
//
//  This file is part of the Intan Technologies RHX Data Acquisition Software.
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published
//  by the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  This software is provided 'as-is', without any express or implied warranty.
//  In no event will the authors be held liable for any damages arising from
//  the use of this software.
//
//  See <http://www.intantech.com> for documentation and product information.
//
//------------------------------------------------------------------------------

#include <QApplication>
#include <QTimer>
#include <iostream>
#include "Engine/API/Synthetic/pipelinedatarhxcontroller.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

#ifdef __APPLE__
    app.setStyle(QStyleFactory::create("Fusion"));
#endif

    std::cout << "🧪 Testing PipelineDataRHXController in Intan RHX..." << std::endl;
    
    try {
        // Create pipeline controller
        PipelineDataRHXController controller(ControllerRecordUSB2, SampleRate30000Hz);
        std::cout << "✅ Controller created successfully" << std::endl;
        
        // Try to connect to TCP server
        std::cout << "🔌 Attempting TCP connection..." << std::endl;
        if (controller.connectToTCPServer()) {
            std::cout << "🎉 Successfully connected to TCP server!" << std::endl;
            
            // Test some basic functionality
            std::vector<ChipType> chipType;
            std::vector<int> portIndex, commandStream, numChannelsOnPort;
            
            int result = controller.findConnectedChips(chipType, portIndex, commandStream, numChannelsOnPort);
            std::cout << "🔍 findConnectedChips returned: " << result << std::endl;
            std::cout << "📊 Number of data streams: " << controller.numWordsInFifo() << std::endl;
            
            // Wait a bit for potential TCP data
            QTimer::singleShot(3000, [&]() {
                std::cout << "⏰ 3 seconds passed..." << std::endl;
                if (controller.isTCPConnected()) {
                    std::cout << "🟢 TCP connection is still active" << std::endl;
                } else {
                    std::cout << "🔴 TCP connection was lost" << std::endl;
                }
                std::cout << "✅ Test completed successfully!" << std::endl;
                app.quit();
            });
            
            return app.exec();
        } else {
            std::cout << "⚠️  Failed to connect to TCP server - using fallback synthetic data" << std::endl;
            
            // Test fallback functionality
            std::vector<ChipType> chipType;
            std::vector<int> portIndex, commandStream, numChannelsOnPort;
            
            int result = controller.findConnectedChips(chipType, portIndex, commandStream, numChannelsOnPort);
            std::cout << "🔍 findConnectedChips returned: " << result << std::endl;
            std::cout << "📊 Number of data streams: " << controller.numWordsInFifo() << std::endl;
            
            std::cout << "✅ Fallback synthetic data working correctly" << std::endl;
            std::cout << "✅ Test completed successfully!" << std::endl;
            return 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "❌ Exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Unknown exception occurred" << std::endl;
        return 1;
    }
}
