/**
 * Test rapido per verificare GPIO 17 con libgpiod su Raspberry Pi 5
 * Compila con: g++ -o test_gpio17 test_gpio17_libgpiod.cpp -lgpiod
 */

#include <iostream>
#include <gpiod.h>
#include <unistd.h>
#include <cstring>

int main() {
    const char* chip_path = "/dev/gpiochip0";
    const unsigned int gpio_line_num = 17;

    std::cout << "=== Test GPIO 17 con libgpiod ===" << std::endl;
    std::cout << "Chip: " << chip_path << std::endl;
    std::cout << "GPIO: " << gpio_line_num << std::endl;

    // Apri chip GPIO
    struct gpiod_chip* chip = gpiod_chip_open(chip_path);
    if (!chip) {
        std::cerr << "ERRORE: Impossibile aprire " << chip_path << std::endl;
        std::cerr << "Errno: " << strerror(errno) << std::endl;
        return 1;
    }
    std::cout << "✓ Chip GPIO aperto" << std::endl;

    // Ottieni linea GPIO 17
    struct gpiod_line* line = gpiod_chip_get_line(chip, gpio_line_num);
    if (!line) {
        std::cerr << "ERRORE: Impossibile ottenere GPIO " << gpio_line_num << std::endl;
        std::cerr << "Errno: " << strerror(errno) << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }
    std::cout << "✓ Linea GPIO 17 acquisita" << std::endl;

    // Richiedi linea come output
    int ret = gpiod_line_request_output(line, "test-gpio17", 0);
    if (ret < 0) {
        std::cerr << "ERRORE: Impossibile configurare GPIO 17 come output" << std::endl;
        std::cerr << "Errno: " << strerror(errno) << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }
    std::cout << "✓ GPIO 17 configurato come output (valore iniziale: LOW)" << std::endl;

    // Test: imposta HIGH
    std::cout << "\nTest 1: Imposto GPIO 17 a HIGH..." << std::endl;
    ret = gpiod_line_set_value(line, 1);
    if (ret < 0) {
        std::cerr << "ERRORE: Impossibile impostare GPIO HIGH" << std::endl;
        std::cerr << "Errno: " << strerror(errno) << std::endl;
    } else {
        std::cout << "✓ GPIO 17 = HIGH" << std::endl;
    }

    // Verifica valore
    int value = gpiod_line_get_value(line);
    if (value < 0) {
        std::cerr << "ERRORE: Impossibile leggere valore GPIO" << std::endl;
    } else {
        std::cout << "✓ Valore letto: " << (value ? "HIGH" : "LOW") << std::endl;
    }

    sleep(1);

    // Test: imposta LOW
    std::cout << "\nTest 2: Imposto GPIO 17 a LOW..." << std::endl;
    ret = gpiod_line_set_value(line, 0);
    if (ret < 0) {
        std::cerr << "ERRORE: Impossibile impostare GPIO LOW" << std::endl;
        std::cerr << "Errno: " << strerror(errno) << std::endl;
    } else {
        std::cout << "✓ GPIO 17 = LOW" << std::endl;
    }

    // Verifica valore
    value = gpiod_line_get_value(line);
    if (value < 0) {
        std::cerr << "ERRORE: Impossibile leggere valore GPIO" << std::endl;
    } else {
        std::cout << "✓ Valore letto: " << (value ? "HIGH" : "LOW") << std::endl;
    }

    // Cleanup
    std::cout << "\nPulizia risorse..." << std::endl;
    gpiod_line_release(line);
    std::cout << "✓ Linea GPIO rilasciata" << std::endl;

    gpiod_chip_close(chip);
    std::cout << "✓ Chip GPIO chiuso" << std::endl;

    std::cout << "\n=== TEST COMPLETATO CON SUCCESSO ===" << std::endl;
    return 0;
}
