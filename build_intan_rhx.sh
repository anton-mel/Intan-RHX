#!/bin/bash

# Author: Anton Melnychuk
# Date: 2025-08-01
# Version: 1.0.0
# Description: This script compiles the Intan RHX application and creates a proper macOS app bundle
# Usage: ./build_intan_rhx.sh
# Dependencies: Qt6, OpenCL, Xcode Command Line Tools

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to check dependencies
check_dependencies() {
    print_status "Checking build dependencies..."
    
    local missing_deps=()
    
    # Check for Qt6
    if ! command_exists qmake6; then
        if ! command_exists qmake; then
            missing_deps+=("Qt6 (qmake)")
        else
            # Check if qmake is Qt6
            QT_VERSION=$(qmake -query QT_VERSION 2>/dev/null | cut -d. -f1)
            if [ "$QT_VERSION" != "6" ]; then
                missing_deps+=("Qt6 (found Qt$QT_VERSION)")
            fi
        fi
    fi
    
    # Check for OpenCL
    if [ ! -d "/System/Library/Frameworks/OpenCL.framework" ]; then
        missing_deps+=("OpenCL framework")
    fi
    
    # Check for Xcode command line tools
    if ! command_exists clang; then
        missing_deps+=("Xcode Command Line Tools")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing dependencies:"
        for dep in "${missing_deps[@]}"; do
            echo "  - $dep"
        done
        echo ""
        print_status "To install dependencies:"
        echo "  1. Install Xcode Command Line Tools:"
        echo "     xcode-select --install"
        echo "  2. Install Qt6 (using Homebrew):"
        echo "     brew install qt@6"
        echo "  3. OpenCL is included with macOS"
        exit 1
    fi
    
    print_success "All dependencies found!"
}

# Function to clean previous builds
clean_build() {
    print_status "Cleaning previous build artifacts..."
    
    if [ -d "build" ]; then
        rm -rf build
    fi
    
    if [ -f "Makefile" ]; then
        make clean 2>/dev/null || true
    fi
    
    print_success "Build directory cleaned"
}

# Function to create build directory
setup_build() {
    print_status "Setting up build directory..."
    
    mkdir -p build
    cd build
    
    print_success "Build directory created"
}

# Function to configure the project
configure_project() {
    print_status "Configuring project with qmake..."
    
    # Use qmake6 if available, otherwise try qmake
    if command_exists qmake6; then
        QMAKE_CMD="qmake6"
    else
        QMAKE_CMD="qmake"
    fi
    
    # Configure the project
    $QMAKE_CMD ../IntanRHX.pro
    
    if [ $? -ne 0 ]; then
        print_error "qmake configuration failed"
        exit 1
    fi
    
    print_success "Project configured successfully"
}

# Function to compile the project
compile_project() {
    print_status "Compiling Intan RHX..."
    
    # Get number of CPU cores for parallel compilation (limit to 4 to avoid file system conflicts)
    CORES=$(sysctl -n hw.ncpu)
    if [ $CORES -gt 4 ]; then
        CORES=4
    fi
    print_status "Using $CORES CPU cores for compilation"
    
    # Compile with make
    make -j$CORES || {
        print_warning "Parallel compilation failed, trying single-threaded compilation..."
        make -j1
    }
    
    if [ $? -ne 0 ]; then
        print_error "Compilation failed"
        exit 1
    fi
    
    print_success "Compilation completed successfully"
}

# Function to create macOS app bundle
create_app_bundle() {
    print_status "Creating macOS app bundle..."
    
    # Create app bundle structure
    mkdir -p IntanRHX.app/Contents/MacOS
    mkdir -p IntanRHX.app/Contents/Frameworks
    mkdir -p IntanRHX.app/Contents/Resources
    
    # Copy the executable (it was already created in the app bundle by the linker)
    if [ ! -f "IntanRHX.app/Contents/MacOS/IntanRHX" ]; then
        print_error "Executable not found in app bundle"
        exit 1
    fi
    
    # Copy required libraries
    cp ../libraries/Mac/libokFrontPanel.dylib IntanRHX.app/Contents/Frameworks/
    
    # Copy required files
    cp ../kernel.cl IntanRHX.app/Contents/Resources/
    cp ../FPGA-bitfiles/*.bit IntanRHX.app/Contents/Resources/
    
    # Create Info.plist
    cat > IntanRHX.app/Contents/Info.plist << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>IntanRHX</string>
    <key>CFBundleIdentifier</key>
    <string>com.intantech.intanrhx</string>
    <key>CFBundleName</key>
    <string>Intan RHX</string>
    <key>CFBundleDisplayName</key>
    <string>Intan RHX</string>
    <key>CFBundleVersion</key>
    <string>1.0</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0</string>
    <key>CFBundlePackageType</key>
    <string>APPL</string>
    <key>CFBundleSignature</key>
    <string>????</string>
    <key>LSMinimumSystemVersion</key>
    <string>10.15</string>
    <key>NSHighResolutionCapable</key>
    <true/>
    <key>NSRequiresAquaSystemAppearance</key>
    <false/>
</dict>
</plist>
EOF
    
    # Use macdeployqt to handle Qt dependencies
    if command_exists macdeployqt; then
        print_status "Running macdeployqt to handle Qt dependencies..."
        macdeployqt IntanRHX.app
    else
        print_warning "macdeployqt not found. Qt dependencies may need to be installed manually."
    fi
    
    print_success "App bundle created: IntanRHX.app"
}

# Function to set up runtime environment
setup_runtime() {
    print_status "Setting up runtime environment..."
    
    # Make the app executable
    chmod +x IntanRHX.app/Contents/MacOS/IntanRHX
    
    # Set up library paths
    install_name_tool -add_rpath "@executable_path/../Frameworks" IntanRHX.app/Contents/MacOS/IntanRHX 2>/dev/null || true
    
    # Code sign the app to avoid macOS security issues
    print_status "Code signing the application..."
    codesign --force --deep --sign - IntanRHX.app || {
        print_warning "Code signing failed, but continuing..."
    }
    
    print_success "Runtime environment configured"
}

# Function to test the build
test_build() {
    print_status "Testing the build..."
    
    if [ -f "IntanRHX.app/Contents/MacOS/IntanRHX" ]; then
        print_success "Build test passed - executable found"
    else
        print_error "Build test failed - executable not found"
        exit 1
    fi
}

# Function to move app to current directory
move_app_to_current_dir() {
    print_status "Moving app bundle to current directory..."
    
    # Move the app bundle to the original directory
    mv IntanRHX.app "$ORIGINAL_DIR/"
    
    # Return to original directory
    cd "$ORIGINAL_DIR"
    
    # Clean up build directory
    rm -rf build
    
    print_success "App moved to: $(pwd)/IntanRHX.app"
}

# Function to automatically open the app
auto_open_app() {
    print_status "Automatically opening IntanRHX.app..."
    
    if [ -d "IntanRHX.app" ]; then
        open IntanRHX.app
        print_success "IntanRHX.app opened successfully!"
    else
        print_error "IntanRHX.app not found - cannot open automatically"
    fi
}

# Function to display final instructions
show_final_instructions() {
    echo ""
    print_success "Build completed successfully!"
    echo ""
    echo "Your Intan RHX application is ready at:"
    echo "  $(pwd)/IntanRHX.app"
    echo ""
    echo "The application should have opened automatically."
    echo "If it didn't open, you can:"
    echo "  1. Double-click IntanRHX.app in Finder"
    echo "  2. Or run from terminal: open IntanRHX.app"
    echo ""
    echo "Note: The first time you run the app, macOS may ask for permission"
    echo "to run the application. You may need to go to System Preferences >"
    echo "Security & Privacy and allow the application to run."
    echo ""
}

# Main build process
main() {
    echo "=========================================="
    echo "    Intan RHX Build Script for macOS"
    echo "=========================================="
    echo ""
    
    # Store original directory
    ORIGINAL_DIR=$(pwd)
    
    # Check dependencies
    check_dependencies
    
    # Clean previous builds
    clean_build
    
    # Setup and build
    setup_build
    configure_project
    compile_project
    
    # Create app bundle
    create_app_bundle
    setup_runtime
    
    # Test the build
    test_build
    
    # Move app to current directory
    move_app_to_current_dir
    
    # Automatically open the app
    auto_open_app
    
    # Show final instructions
    show_final_instructions
}

# Run the main function
main "$@"
