/**
 * @file c_api_example.c
 * @brief C API usage example for the Unlook 3D Scanner
 * 
 * This example demonstrates the C-style API functions that can be
 * used for language bindings or C-only applications.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // For sleep()

// Include the C API functions (declared in unlook_scanner.h)
// In a real application, you would include the appropriate header
extern void* unlook_scanner_create(void);
extern void unlook_scanner_destroy(void* scanner);
extern int unlook_scanner_initialize(void* scanner, int mode, const char* config_path);
extern int unlook_scanner_shutdown(void* scanner);
extern int unlook_scanner_is_initialized(void* scanner);
extern const char* unlook_get_version(void);

int main() {
    printf("=== Unlook 3D Scanner API - C Example ===\n");
    printf("Version: %s\n", unlook_get_version());
    printf("\n");
    
    // Create scanner instance
    printf("Creating scanner instance...\n");
    void* scanner = unlook_scanner_create();
    
    if (scanner == NULL) {
        fprintf(stderr, "Failed to create scanner instance\n");
        return 1;
    }
    
    printf("Scanner instance created successfully\n");
    
    // Check initial state
    int is_initialized = unlook_scanner_is_initialized(scanner);
    printf("Initial state - Initialized: %s\n", is_initialized ? "Yes" : "No");
    
    // Initialize scanner in standalone mode
    printf("\nInitializing scanner (standalone mode)...\n");
    int result = unlook_scanner_initialize(scanner, 0, NULL); // 0 = STANDALONE, NULL = default config
    
    if (result != 0) {
        fprintf(stderr, "Scanner initialization failed with code: %d\n", result);
        unlook_scanner_destroy(scanner);
        return 1;
    }
    
    printf("Scanner initialized successfully!\n");
    
    // Verify initialization
    is_initialized = unlook_scanner_is_initialized(scanner);
    printf("After initialization - Initialized: %s\n", is_initialized ? "Yes" : "No");
    
    // Simulate some work
    printf("\nRunning scanner for 3 seconds...\n");
    sleep(3);
    
    // Shutdown scanner
    printf("\nShutting down scanner...\n");
    result = unlook_scanner_shutdown(scanner);
    
    if (result != 0) {
        fprintf(stderr, "Scanner shutdown failed with code: %d\n", result);
    } else {
        printf("Scanner shutdown complete\n");
    }
    
    // Verify shutdown
    is_initialized = unlook_scanner_is_initialized(scanner);
    printf("After shutdown - Initialized: %s\n", is_initialized ? "Yes" : "No");
    
    // Destroy scanner instance
    printf("\nDestroying scanner instance...\n");
    unlook_scanner_destroy(scanner);
    
    printf("C API example completed successfully!\n");
    return 0;
}

/*
 * Compilation instructions:
 * 
 * gcc -o c_api_example c_api_example.c -lunlook -I/path/to/unlook/include
 * 
 * Or using pkg-config:
 * gcc -o c_api_example c_api_example.c `pkg-config --cflags --libs unlook`
 * 
 * Example usage in other languages:
 * 
 * Python (using ctypes):
 * ```python
 * import ctypes
 * 
 * # Load the library
 * lib = ctypes.CDLL('libunlook.so')
 * 
 * # Define function signatures
 * lib.unlook_scanner_create.restype = ctypes.c_void_p
 * lib.unlook_scanner_initialize.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p]
 * lib.unlook_scanner_initialize.restype = ctypes.c_int
 * 
 * # Use the API
 * scanner = lib.unlook_scanner_create()
 * result = lib.unlook_scanner_initialize(scanner, 0, None)
 * # ... rest of operations
 * lib.unlook_scanner_destroy(scanner)
 * ```
 * 
 * JavaScript (using Node.js ffi):
 * ```javascript
 * const ffi = require('ffi-napi');
 * const ref = require('ref-napi');
 * 
 * const lib = ffi.Library('libunlook.so', {
 *   'unlook_scanner_create': ['pointer', []],
 *   'unlook_scanner_initialize': ['int', ['pointer', 'int', 'string']],
 *   'unlook_scanner_destroy': ['void', ['pointer']]
 * });
 * 
 * const scanner = lib.unlook_scanner_create();
 * const result = lib.unlook_scanner_initialize(scanner, 0, null);
 * // ... operations
 * lib.unlook_scanner_destroy(scanner);
 * ```
 * 
 * Go (using CGO):
 * ```go
 * package main
 * 
 * // #cgo LDFLAGS: -lunlook
 * // #include <unlook/api/unlook_scanner.h>
 * import "C"
 * import "fmt"
 * 
 * func main() {
 *     scanner := C.unlook_scanner_create()
 *     defer C.unlook_scanner_destroy(scanner)
 *     
 *     result := C.unlook_scanner_initialize(scanner, 0, nil)
 *     if result != 0 {
 *         fmt.Printf("Initialization failed: %d\n", result)
 *         return
 *     }
 *     
 *     // ... operations
 *     C.unlook_scanner_shutdown(scanner)
 * }
 * ```
 */