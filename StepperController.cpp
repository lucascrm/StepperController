/**
 * @file StepperController.cpp
 * @brief Implementación del controlador para motores paso a paso.
 * @version 1.0
 * @author L. Cruz
 * @email lucascrm@gmail.com
 */


#include "StepperController.h"

#include <algorithm>
#include <stdexcept>
#include <pigpio.h>

#define PI_LOW	0
#define PI_HIGH	1
#define STEP_PULSE_WIDTH_US 2 // Ancho de pulso STEP en microsegundos

namespace motor {

// ============================================================================
//                        FUNCIONES AUXILIARES (Privadas)
// ============================================================================

/**
 * @brief Convierte RPM a pasos por segundo según el motor configurado.
 * @param rpm Revoluciones por minuto.
 * @return Pasos por segundo equivalentes.
 */
double StepperController::rpmToStepsPerSec(double rpm) const {
    return (rpm * config_.steps_per_rev) / 60.0;
}

/**
 * @brief Convierte pasos por segundo a RPM.
 * @param steps_per_sec Frecuencia actual de pasos.
 * @return RPM equivalentes.
 */
double StepperController::stepsPerSecToRpm(double steps_per_sec) const {
    return (steps_per_sec * 60.0) / config_.steps_per_rev;
}

/**
 * @brief Obtiene la velocidad actual del motor en RPM.
 * @return Velocidad calculada en RPM.
 */
double StepperController::getCurrentRPM() const {
    return stepsPerSecToRpm(current_speed_steps_per_sec_.load());
}

// ============================================================================
//                        CONSTRUCTOR / DESTRUCTOR
// ============================================================================

/**
 * @brief Constructor del controlador de motor paso a paso.
 *        Configura los pines GPIO e inicia el hilo de control.
 * @param config Configuración del motor (pines, pasos/rev, aceleración, etc.)
 */
StepperController::StepperController(const MotorConfig& config)
    : config_(config), current_position_(0), target_position_(0),
      is_moving_(false), enabled_(false), running_(true),
      target_speed_steps_per_sec_(0.0), current_speed_steps_per_sec_(0.0),
      current_direction_(0), is_homing_(false) {

    // Configurar pines GPIO
    gpioSetMode(config_.step_pin, PI_OUTPUT);
    gpioSetMode(config_.dir_pin, PI_OUTPUT);
    
    if (config_.enable_pin != -1) {
        gpioSetMode(config_.enable_pin, PI_OUTPUT);
        // Inicializar en estado deshabilitado (Active low para la mayoría)
        gpioWrite(config_.enable_pin, PI_HIGH); 
    }

    // Inicializar pines de control en LOW
    gpioWrite(config_.step_pin, PI_LOW);
    gpioWrite(config_.dir_pin, PI_LOW);

    // Iniciar hilo de control unificado (CORRECCIÓN CRÍTICA)
    control_thread_ = std::thread(&StepperController::stepControlThread, this);

    std::cout << "StepperController inicializado en pines: "
              << "STEP=" << config_.step_pin
              << ", DIR=" << config_.dir_pin
              << ", ENABLE=" << config_.enable_pin << std::endl;
}

/**
 * @brief Destructor. Detiene el hilo y deshabilita el motor.
 */
StepperController::~StepperController() {
    running_ = false;

    if (control_thread_.joinable()) 
	    control_thread_.join();

    disable();
}

// ============================================================================
//                               CONTROL BÁSICO
// ============================================================================

/**
 * @brief Mueve el motor una cantidad fija de pasos.
 * @param steps Número de pasos a mover.
 * @param direction Dirección (1 = adelante, -1 = atrás).
 *
 * Configura el objetivo de posición y deja que el hilo de control
 * se encargue de la aceleración, velocidad y parada.
 */
void StepperController::moveSteps(long steps, int direction) {
    if (!enabled_) 
	    enable();

    // Bloqueo del mutex para configurar la nueva posición objetivo
    std::lock_guard<std::mutex> lock(state_mutex_);

    // La dirección se establece aquí y se usa en stepControlThread
    current_direction_ = (direction > 0) ? 1 : -1;
    gpioWrite(config_.dir_pin, current_direction_ > 0 ? PI_HIGH : PI_LOW);

    // Si ya estamos moviéndonos, movemos la posición objetivo
    target_position_ = current_position_.load() + (steps * current_direction_);
    
    // Usamos la velocidad máxima por defecto para la lógica simple
    target_speed_steps_per_sec_ = rpmToStepsPerSec(config_.max_rpm);
    is_moving_ = true;

    std::cout << "Movimiento programado: " << steps
              << " pasos, dirección: " << current_direction_
              << ", posición objetivo: " << target_position_ << std::endl;
}

/**
 * @brief Mueve el motor un número dado de revoluciones.
 * @param revolutions Número de revoluciones.
 * @param direction Sentido de giro.
 */
void StepperController::moveRevolutions(double revolutions, int direction) {
    long steps = static_cast<long>(revolutions * config_.steps_per_rev);
    moveSteps(steps, direction);
}

/**
 * @brief Detiene el motor inmediatamente sin desacelerar.
 */
void StepperController::stop() {
    // Detención inmediata, sin desaceleración
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_position_.store(current_position_.load()); // Target = Posición actual
    target_speed_steps_per_sec_ = 0.0;
    current_speed_steps_per_sec_ = 0.0;
    current_direction_ = 0;
    is_moving_ = false;
    is_homing_ = false;
}

/**
 * @brief Activa el motor (Active-low).
 */
void StepperController::enable() { // Active low
    if (config_.enable_pin != -1) {
        gpioWrite(config_.enable_pin, PI_LOW);
        enabled_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
    }
}

/**
 * @brief Desactiva el motor y detiene el movimiento.
 */
void StepperController::disable() {
    stop();
    if (config_.enable_pin != -1) {
        gpioWrite(config_.enable_pin, PI_HIGH);
        enabled_ = false;
    }
}

/**
 * @brief Resetea el contador interno de pasos.
 */
void StepperController::resetPosition() {
    current_position_ = 0L;
}

// ============================================================================
//                              CONTROL DE VELOCIDAD
// ============================================================================

/**
 * @brief Configura la velocidad objetivo en RPM.
 * @param rpm Velocidad deseada.
 */
void StepperController::setSpeedRPM(double rpm) {
    rpm = std::min(rpm, static_cast<double>(config_.max_rpm));
    target_speed_steps_per_sec_ = rpmToStepsPerSec(rpm);

    std::cout << "Velocidad objetivo configurada: " << rpm << " RPM ("
              << target_speed_steps_per_sec_.load() << " pasos/segundo)" << std::endl;
}

/**
 * @brief Configura la velocidad en pasos por segundo.
 * @param steps_per_sec Frecuencia deseada.
 */
void StepperController::setSpeedStepsPerSecond(double steps_per_sec) {
    double rpm = stepsPerSecToRpm(steps_per_sec);
    setSpeedRPM(rpm);
}

// ============================================================================
//                            CONTROL AVANZADO
// ============================================================================

/**
 * @brief Inicia un movimiento hacia un objetivo con aceleración controlada.
 * @param target_steps Nueva posición objetivo absoluta.
 * @param rpm Velocidad máxima permitida.
 */
void StepperController::moveWithAcceleration(long target_steps, double rpm) {
    if (!enabled_) 
	    enable();

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    target_speed_steps_per_sec_ = rpmToStepsPerSec(rpm);
    target_position_ = target_steps;
    current_direction_ = (target_steps > current_position_.load()) ? 1 : -1;
    gpioWrite(config_.dir_pin, current_direction_ > 0 ? PI_HIGH : PI_LOW);
    
    is_moving_ = true;
}

/**
 * @brief Movimiento con rampas personalizadas.
 * @param steps Pasos a mover.
 * @param rpm Velocidad máxima.
 * @param accel Aceleración.
 * @param decel Desaceleración.
 */
void StepperController::smoothMove(long steps, double rpm, int accel, int decel) {
    config_.acceleration = accel;
    config_.deceleration = decel;

    moveWithAcceleration(current_position_.load() + steps, rpm);
}

/**
 * @brief Inicia un movimiento continuo sin posición final.
 * @param rpm Velocidad constante.
 * @param direction Dirección de giro.
 */
void StepperController::startContinuousMove(double rpm, int direction) {
    if (!enabled_) 
	    enable();

    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // Configurar la dirección
    current_direction_ = (direction > 0) ? 1 : -1;
    gpioWrite(config_.dir_pin, current_direction_ > 0 ? PI_HIGH : PI_LOW);

    // Establecer target de posición virtualmente infinito para evitar deceleración automática
    // Usamos la mitad del límite para evitar overflow en el hilo de control
    target_position_ = (current_direction_ == 1) ? 
                       std::numeric_limits<long>::max() / 2 : 
                       std::numeric_limits<long>::min() / 2;
    
    // Configurar velocidad objetivo
    double speed_steps_per_sec = rpmToStepsPerSec(rpm);
    target_speed_steps_per_sec_ = speed_steps_per_sec;

    // Si el motor está parado, acelerar desde 0
    if (current_speed_steps_per_sec_ < 0.1) {
        current_speed_steps_per_sec_ = 0.0;
    }
    
    is_moving_ = true;

    std::cout << "Rotación continua programada a " << rpm << " RPM, dirección: " 
              << current_direction_ << std::endl;
}

// ============================================================================
//                     HILO DE CONTROL DE MOVIMIENTO (Privado)
// ============================================================================

/**
 * @brief Genera un único pulso STEP y actualiza la posición.
 * @param direction Dirección del paso (+1 o -1).
 */
void StepperController::generateStep(int direction) {
    // Generar pulso STEP
    gpioWrite(config_.step_pin, PI_HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(STEP_PULSE_WIDTH_US)); 
    gpioWrite(config_.step_pin, PI_LOW);

    // Actualizar posición
    current_position_ += direction;
    last_step_time_ = std::chrono::steady_clock::now();
}

/**
 * @brief Hilo continuo encargado de:
 * - Aceleración y desaceleración
 * - Generación de pasos
 * - Control de límites (endstops)
 * - Detección de objetivo alcanzado
 */
void StepperController::stepControlThread() {
    std::cout << "Thread de control de pasos y aceleración iniciado para pin " << config_.step_pin << std::endl;

    while (running_) {
        
        // Espera si el motor no debe moverse
        if (!is_moving_.load() || !enabled_.load()) {
            current_speed_steps_per_sec_ = 0.0;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        
        // Lectura de variables atómicas
        double current_speed = current_speed_steps_per_sec_.load();
        double target_speed = target_speed_steps_per_sec_.load();
        long current_pos = current_position_.load();
        long target_pos = target_position_.load();
        int direction = current_direction_.load();
        
        // 1. Verificación de parada (Target alcanzado en movimiento limitado)
        if (direction != 0 &&
            ((direction > 0 && current_pos >= target_pos) || 
             (direction < 0 && current_pos <= target_pos))) {
            
            // Si no estamos en Homing y alcanzamos el target, detenemos
            if (!is_homing_.load()) {
                stop();
                continue;
            }
        }
        
        // 2. Control de Endstops (para cualquier movimiento)
        if (is_moving_.load()) {
            if (direction > 0 && max_limit_pin_ != -1 && gpioRead(max_limit_pin_) == PI_LOW) {
                std::cout << "Endstop MAX alcanzado. Deteniendo." << std::endl;
                stop();
                continue;
            }
            if (direction < 0 && min_limit_pin_ != -1 && gpioRead(min_limit_pin_) == PI_LOW) {
                std::cout << "Endstop MIN alcanzado. Deteniendo." << std::endl;
                stop();
                
                // Si estamos en homing, reseteamos la posición
                if (is_homing_.load()) {
                    current_position_ = 0;
                    std::cout << "Homing completado. Posición resetada a 0." << std::endl;
                }
                continue;
            }
        }

        // 3. Lógica de Aceleración/Desaceleración (CORRECCIÓN CRÍTICA)
        double accel_rate = static_cast<double>(config_.acceleration); // steps/s²
        double decel_rate = static_cast<double>(config_.deceleration); // steps/s²
        double dt = 0.001; // Asumiendo un paso de tiempo de 1ms para cálculo (esto es una simplificación)
        
        double next_speed = current_speed;
        
        // Distancia necesaria para detenerse (solo relevante si el target es finito)
        double distance_to_go = std::abs(target_pos - current_pos);
        double needed_decel_dist = (current_speed * current_speed) / (2.0 * decel_rate);
        
        bool should_decelerate_for_target = distance_to_go < needed_decel_dist && distance_to_go < (long)std::numeric_limits<long>::max() / 4; 
        
        if (should_decelerate_for_target) {
            // Desacelerar para detenerse en el target
            next_speed = std::max(0.0, current_speed - (decel_rate * dt));
            
        } else if (current_speed < target_speed) {
            // Acelerar a la velocidad objetivo
            next_speed = std::min(target_speed, current_speed + (accel_rate * dt));
            
        } else if (current_speed > target_speed) {
            // Desacelerar a la velocidad objetivo
            next_speed = std::max(target_speed, current_speed - (decel_rate * dt));
        }

        current_speed_steps_per_sec_ = next_speed;

        // 4. Generación del Paso
        if (next_speed > 0.0) {
            unsigned long delay_us = static_cast<unsigned long>(1000000.0 / next_speed);
            
            auto now = std::chrono::steady_clock::now();
            auto time_since_last_step = std::chrono::duration_cast<std::chrono::microseconds>(now - last_step_time_).count();

            if (time_since_last_step >= (long)delay_us) {
                generateStep(direction);
            } else {
                // Espera precisa
                long remaining_delay = (long)delay_us - time_since_last_step;
                if (remaining_delay > 0) {
                     // Solo duerme por el resto del delay necesario
                     std::this_thread::sleep_for(std::chrono::microseconds(remaining_delay));
                }
            }
        } else {
            // Espera si la velocidad es 0 (pero is_moving_ es true)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// ============================================================================
//                        ENDSTOPS & HOMING
// ============================================================================

/**
 * @brief Configura los pines de endstop (mínimo y máximo).
 * @param min_pin Pin del endstop mínimo (Active-low).
 * @param max_pin Pin del endstop máximo (Active-low).
 */
void StepperController::setLimitSwitches(int min_pin, int max_pin) {
    min_limit_pin_ = min_pin;
    max_limit_pin_ = max_pin;

    if (min_pin != -1) {
	    gpioSetMode(min_pin, PI_INPUT);
	    gpioSetPullUpDown(min_pin, PI_PUD_UP); // Asume Active Low
    }

    if (max_pin != -1) {
	    gpioSetMode(max_pin, PI_INPUT);
	    gpioSetPullUpDown(max_pin, PI_PUD_UP); // Asume Active Low
    }
}

/**
 * @brief Ejecuta un proceso de homing hacia un endstop.
 * @param direction Sentido del homing.
 * @param homing_speed Velocidad durante el homing (RPM).
 *
 * El método bloquea hasta que el hilo de control alcanza el endstop
 * y resetea la posición a cero.
 */
void StepperController::home(int direction, double homing_speed) {
    if (min_limit_pin_ == -1 && max_limit_pin_ == -1) {
        std::cerr << "No hay endstops configurados para homing" << std::endl;
        return;
    }

    // Bloquea el thread principal para esperar el resultado del hilo de control
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (is_moving_.load()) {
        std::cerr << "Error: No se puede iniciar homing mientras el motor está en movimiento." << std::endl;
        return;
    }

    std::cout << "Iniciando homing..." << std::endl;
    enable();
    
    is_homing_ = true;
    current_direction_ = (direction > 0) ? 1 : -1;
    gpioWrite(config_.dir_pin, current_direction_ > 0 ? PI_HIGH : PI_LOW);
    
    target_speed_steps_per_sec_ = rpmToStepsPerSec(homing_speed);
    
    // Establecer un target de posición muy lejano (movimiento indefinido)
    target_position_ = (current_direction_ == 1) ? 
                       std::numeric_limits<long>::max() / 2 : 
                       std::numeric_limits<long>::min() / 2;
    is_moving_ = true;
    
    // El hilo principal espera a que el control_thread se detenga al detectar el límite.
    while (is_moving_.load() && running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    // stop() ya se llama dentro del stepControlThread al detectar el límite,
    // pero lo llamamos de nuevo si el loop terminó por otra razón.
    stop(); 
    is_homing_ = false;

    // La posición se resetea a 0 dentro del stepControlThread cuando detecta el límite.
    if (current_position_.load() == 0) {
        std::cout << "Homing completado. Posición resetada a 0." << std::endl;
    } else {
        std::cout << "Homing detenido sin alcanzar el límite. Posición actual: " << current_position_.load() << std::endl;
    }
}


} // namespace motor
