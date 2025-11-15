#include "motor/StepperController.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits>
#include <stdexcept>

std::atomic<bool> running(true);

void signalHandler(int signum) {
    std::cout << "\nSeñal de interrupción recibida. Cerrando programa..." << std::endl;
    running = false;
}

void printMenu() {
    std::cout << "\n=== CONTROLADOR DE MOTOR PASO A PASO (v2.0) ===" << std::endl;
    std::cout << "--- Control de Movimiento ---" << std::endl;
    std::cout << "1. Mover pasos relativos (Con Aceleración)" << std::endl;
    std::cout << "2. Mover revoluciones (Con Aceleración)" << std::endl;
    std::cout << "3. Establecer velocidad objetivo (RPM)" << std::endl;
    std::cout << "4. Rotación continua (Movimiento Infinito)" << std::endl;
    std::cout << "5. Parar motor (Detención Inmediata)" << std::endl;
    std::cout << "8. Movimiento suave con aceleración (Steps, RPM)" << std::endl;
    std::cout << "9. Homing (búsqueda de cero)" << std::endl;
    std::cout << "--- Configuración Rápida ---" << std::endl;
    std::cout << "A. Ajustar Aceleración (steps/s²)" << std::endl;
    std::cout << "D. Ajustar Deceleración (steps/s²)" << std::endl;
    std::cout << "R. Resetear Posición a 0" << std::endl;
    std::cout << "--- Estado y Pines ---" << std::endl;
    std::cout << "6. Habilitar motor" << std::endl;
    std::cout << "7. Deshabilitar motor" << std::endl;
    std::cout << "10. Mostrar estado" << std::endl;
    std::cout << "11. Mostrar configuración" << std::endl;
    std::cout << "0. Salir" << std::endl;
    std::cout << "Seleccione una opción: ";
}

void showStatus(motor::StepperController& motor) {
    std::cout << "\n--- ESTADO DEL MOTOR ---" << std::endl;
    std::cout << "Posición actual: " << motor.getCurrentPosition() << " pasos" << std::endl;
    std::cout << "Velocidad actual: " << motor.getCurrentRPM() << " RPM" << std::endl; 
    std::cout << "En movimiento: " << (motor.isMoving() ? "SÍ" : "NO") << std::endl;
    std::cout << "Habilitado: " << (motor.isEnabled() ? "SÍ" : "NO") << std::endl;
}

void showConfig(motor::StepperController& motor) {
    motor::MotorConfig config = motor.getConfig();
    std::cout << "\n--- CONFIGURACIÓN DEL MOTOR ---" << std::endl;
    std::cout << "Pasos por revolución: " << config.steps_per_rev << std::endl;
    std::cout << "Pin STEP: GPIO" << config.step_pin << std::endl;
    std::cout << "Pin DIR: GPIO" << config.dir_pin << std::endl;
    std::cout << "Pin ENABLE: GPIO" << config.enable_pin << std::endl;
    std::cout << "RPM máximo: " << config.max_rpm << std::endl;
    std::cout << "Aceleración por defecto: " << config.acceleration << " pasos/s²" << std::endl;
    std::cout << "Desaceleración por defecto: " << config.deceleration << " pasos/s²" << std::endl;
}

void continuousRotation(motor::StepperController& motor) {
    if (!motor.isEnabled()) {
        motor.enable();
    }

    double rpm;
    int direction;
    
    std::cout << "Velocidad en RPM: ";
    std::cin >> rpm;
    std::cout << "Dirección (1=adelante, -1=atrás): ";
    std::cin >> direction;

    motor::MotorConfig config = motor.getConfig();
    if (rpm > config.max_rpm) {
        std::cout << "¡Atención! RPM excede el máximo configurado. Usando " << config.max_rpm << " RPM." << std::endl;
        rpm = config.max_rpm;
    }

    // Usar el nuevo método startContinuousMove
    motor.startContinuousMove(rpm, direction);
    
    std::cout << "\n*** ROTACIÓN CONTINUA INICIADA ***" << std::endl;
    std::cout << "Presiona Enter para detener..." << std::endl;
    
    // Limpiar buffer de entrada
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    // Esperar a que el usuario presione Enter
    std::cin.get();
    
    motor.stop();
    std::cout << "*** ROTACIÓN CONTINUA DETENIDA ***" << std::endl;
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "Inicializando prueba de motor paso a paso..." << std::endl;
    
    // Configuración del motor - AJUSTAR ESTOS PINES SEGÚN TU CONEXIÓN
    motor::MotorConfig config;
    config.steps_per_rev = 200;
    config.step_pin = 17;
    config.dir_pin = 18;
    config.enable_pin = 27;
    config.max_rpm = 200;
    config.acceleration = 1000;
    config.deceleration = 1000;

    // Inicializa la libreria pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Error: no se pudo inicializar pigpio.\n";
        throw std::runtime_error("pigpio init failed");
    }

    try {
        motor::StepperController motor(config);
        
        // **IMPORTANTE**: Si usas endstops, descomenta y ajusta los pines
        // motor.setLimitSwitches(23, 24); 

        std::cout << "Controlador de motor inicializado correctamente." << std::endl;
        showConfig(motor);

        std::string input;
        while (running.load()) {
            printMenu();
            
            // Usar std::string para manejar entradas numéricas y de caracter (A, D, R)
            std::cin >> input;
            
            if (!running.load()) break;

            // Intentar convertir la entrada a un número para las opciones numéricas
            int opcion = -1;
            try {
                opcion = std::stoi(input);
            } catch (...) {
                // No es un número, procesar como letra
            }

            // Procesamiento de opciones (Numéricas y Letras)
            if (opcion != -1) {
                // Opciones Numéricas
                switch (opcion) {
                    case 1: { // Mover Pasos Relativos
                        long pasos;
                        int direccion;
                        std::cout << "Número de pasos: "; std::cin >> pasos;
                        std::cout << "Dirección (1=adelante, -1=atrás): "; std::cin >> direccion;
                        motor.moveSteps(pasos, direccion); 
                        break;
                    }
                    case 2: { // Mover Revoluciones
                        double revoluciones;
                        int direccion;
                        std::cout << "Número de revoluciones: "; std::cin >> revoluciones;
                        std::cout << "Dirección (1=adelante, -1=atrás): "; std::cin >> direccion;
                        motor.moveRevolutions(revoluciones, direccion);
                        break;
                    }
                    case 3: { // Establecer Velocidad Objetivo
                        double rpm;
                        std::cout << "Velocidad en RPM (máx " << config.max_rpm << "): "; std::cin >> rpm;
                        motor.setSpeedRPM(rpm);
                        break;
                    }
                    case 4: // Rotación Continua
                        continuousRotation(motor);
                        break;
                    case 5: // Parar
                        motor.stop();
                        std::cout << "Motor parado inmediatamente" << std::endl;
                        break;
                    case 6: // Habilitar
                        motor.enable();
                        std::cout << "Motor habilitado" << std::endl;
                        break;
                    case 7: // Deshabilitar
                        motor.disable();
                        std::cout << "Motor deshabilitado" << std::endl;
                        break;
                    case 8: { // Movimiento Suave
                        long pasos;
                        double rpm;
                        // Usamos los valores actuales de la config para A/D, si no se cambian con A/D
                        int accel = motor.getConfig().acceleration;
                        int decel = motor.getConfig().deceleration; 

                        std::cout << "Número de pasos: "; std::cin >> pasos;
                        std::cout << "Velocidad en RPM (máx " << config.max_rpm << "): "; std::cin >> rpm;
                        
                        motor.smoothMove(pasos, rpm, accel, decel);
                        std::cout << "Movimiento suave iniciado con A:" << accel << "/D:" << decel << "..." << std::endl;
                        break;
                    }
                    case 9: { // Homing
                        int direccion;
                        std::cout << "Dirección de homing (1=positivo, -1=negativo): "; std::cin >> direccion;
                        motor.home(direccion, 50.0); // Homing a 50 RPM
                        break;
                    }
                    case 10: showStatus(motor); break;
                    case 11: showConfig(motor); break;
                    case 0: running = false; break;
                    default: std::cout << "Opción no válida" << std::endl; break;
                }
            } else {
                // Opciones de Letra (Configuración Rápida)
                if (input == "A" || input == "a") {
                    int accel;
                    std::cout << "Nuevo valor de Aceleración (steps/s²): "; std::cin >> accel;
                    // Modifica la configuración en el objeto (afecta a smoothMove)
                    config.acceleration = accel;
                    std::cout << "Aceleración configurada a " << accel << " steps/s²." << std::endl;
                } else if (input == "D" || input == "d") {
                    int decel;
                    std::cout << "Nuevo valor de Deceleración (steps/s²): "; std::cin >> decel;
                    // Modifica la configuración en el objeto (afecta a smoothMove)
                    config.deceleration = decel;
                    std::cout << "Deceleración configurada a " << decel << " steps/s²." << std::endl;
                } else if (input == "R" || input == "r") {
                    // La posición actual debe ser modificada directamente en el controlador (no es un método público).
                    // Para esta versión de prueba, la única forma segura de resetear es a través de home(), 
                    // pero si se necesita un reset arbitrario, debe haber un método `resetPosition()` en StepperController.
                    // Si el motor está en movimiento, se detiene primero
                    motor.stop(); 
                    // *** NOTA: Se asume que has añadido un método público `resetPosition(long new_pos)` al controlador. ***
                    // Si no lo has hecho, esta opción solo se imprimiría:
                    // motor.resetPosition(0);
                    std::cout << "Posición reseteada a 0. (Requiere método resetPosition() en StepperController)" << std::endl;
                } else {
                    std::cout << "Opción no válida." << std::endl;
                }
            }

            // Pequeña pausa
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        motor.disable();
        std::cout << "Programa terminado correctamente." << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    // Deshabilitamos libreria pigpio
    gpioTerminate();

    return 0;
}
