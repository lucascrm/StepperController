/**
 * @file StepperController.h
 * @brief Controlador para motores paso a paso.
 * @version 1.0
 * @author L. Cruz
 * @email lucascrm@gmail.com
 */
#ifndef _STEPPERCONTROLLER_H_
#define _STEPPERCONTROLLER_H_

#include <pigpio.h>
#include <iostream>
#include <memory>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <cmath>
#include <algorithm> // Añadido para std::max/min
#include <limits> // Añadido para límites de long

namespace motor {

/**
 * @brief Configuración básica del motor paso a paso.
 *
 * Esta estructura contiene los parámetros necesarios para inicializar y operar
 * un motor paso a paso NEMA o similar.
 */
struct MotorConfig {
    int steps_per_rev = 200;    ///< Pasos por revolución (NEMA17 típico)
    int step_pin;               ///< Pin GPIO para el pulso STEP
    int dir_pin;                ///< Pin GPIO para la dirección (DIR)
    int enable_pin;             ///< Pin ENABLE (active-low). Usar -1 para desactivar.
    int max_rpm = 150;          ///< RPM máximas permitidas
    int acceleration = 1000;    ///< Aceleración en pasos/s²
    int deceleration = 1000;    ///< Deceleración en pasos/s²
};

/**
 * @brief Representa un comando de movimiento solicitado al motor.
 */
struct MotionCommand {
    /**
     * @brief Tipos de comando posibles.
     */
    enum Type { 
        MOVE_REL,               ///< Movimiento relativo
        MOVE_ABS,               ///< Movimiento absoluto 
        STOP,                   ///< Detener movimiento
        SET_SPEED               ///< Ajuste de velocidad
    };

    Type type;                  ///< Tipo de comando
    long target_steps;          ///< Número de pasos objetivo
    double target_rpm;          ///< Velocidad objetivo en RPM
    bool relative;              ///< Indica si el movimiento es relativo
};

/**
 * @brief Controlador de un motor paso a paso con aceleración, homing y control continuo.
 *
 * Esta clase implementa:
 * - Movimiento absoluto y relativo
 * - Control de aceleración/desaceleración
 * - Movimiento continuo a RPM fija
 * - Endstops (interruptores de límite)
 * - Función de homing
 * - Seguimiento de velocidad y posición
 *
 * Internamente utiliza un hilo dedicado para el control del motor con tiempos precisos.
 */
class StepperController {
public:

    /**
     * @brief Crea un controlador para un motor paso a paso.
     * @param config Configuración inicial del motor.
     */
    StepperController(const MotorConfig& config);

    /**
     * @brief Destructor seguro que detiene el hilo de control.
     */    
    ~StepperController();
    
    // ---------------------------------------------------------------------
    // CONTROL BÁSICO
    // ---------------------------------------------------------------------

    /**
     * @brief Mueve el motor un número específico de pasos.
     * @param steps Número de pasos a mover.
     * @param direction Dirección del movimiento (1 o -1).
     */
    void moveSteps(long steps, int direction = 1);

    /**
     * @brief Mueve el motor un número de revoluciones.
     * @param revolutions Cantidad de vueltas.
     * @param direction Dirección (1 = adelante, -1 = atrás).
     */
    void moveRevolutions(double revolutions, int direction = 1);

    /**
     * @brief Detiene inmediatamente el movimiento del motor.
     *
     * @note La detención es inmediata, sin desaceleración controlada.
     */
    void stop();

    /**
     * @brief Activa el pin ENABLE del motor.
     */
    void enable();

    /**
     * @brief Desactiva el pin ENABLE y detiene el movimiento.
     */
    void disable();
   
    /**
     * @brief Reinicia el contador interno de posición a cero.
     */
    void resetPosition();  

    // ---------------------------------------------------------------------
    // CONTROL DE VELOCIDAD
    // ---------------------------------------------------------------------

    /**
     * @brief Establece la velocidad objetivo en RPM.
     * @param rpm Revoluciones por minuto.
     */
    void setSpeedRPM(double rpm);

    /**
     * @brief Establece la velocidad objetivo en pasos por segundo.
     * @param steps_per_sec Velocidad en pasos/s.
     */
    void setSpeedStepsPerSecond(double steps_per_sec);
    
    // ---------------------------------------------------------------------
    // MOVIMIENTO AVANZADO
    // ---------------------------------------------------------------------

    /**
     * @brief Movimiento con aceleración controlada a un objetivo.
     * @param target_steps Número de pasos objetivo.
     * @param rpm Velocidad máxima en RPM.
     */
    void moveWithAcceleration(long target_steps, double rpm);

    /**
     * @brief Movimiento suave con aceleración y desaceleración.
     * @param steps Pasos a mover.
     * @param rpm Velocidad máxima.
     * @param accel Aceleración (steps/s²).
     * @param decel Deceleración (steps/s²).
     */
    void smoothMove(long steps, double rpm, int accel = 1000, int decel = 1000);
    
    /**
     * @brief Inicia un movimiento continuo a RPM fija.
     * @param rpm Velocidad objetivo.
     * @param direction Dirección (1 / -1).
     *
     * @note El movimiento no termina hasta llamar a stop().
     */
    void startContinuousMove(double rpm, int direction); 
    
    // ---------------------------------------------------------------------
    // ESTADO DEL MOTOR
    // ---------------------------------------------------------------------

    /**
     * @brief Obtiene la posición actual en pasos.
     */
    long getCurrentPosition() const { return current_position_; }

    /**
     * @brief Establece manualmente la posición actual.
     */
    void setCurrentPosition(long pos) { current_position_ = pos; } 

    /**
     * @brief Retorna la velocidad actual en RPM.
     */
    double getCurrentRPM() const;

    /**
     * @brief Indica si el motor está en movimiento.
     */
    bool isMoving() const { return is_moving_; }

    /**
     * @brief Indica si el motor está habilitado (ENABLE LOW).
     */
    bool isEnabled() const { return enabled_; }
    
    /**
     * @brief Obtiene la configuración interna del motor.
     */
    MotorConfig getConfig() const { return config_; }
    
    // ---------------------------------------------------------------------
    // ENDSTOPS Y HOMING
    // ---------------------------------------------------------------------

    /**
     * @brief Configura los pines GPIO usados como interruptores de límite.
     * @param min_pin Pin límite mínimo.
     * @param max_pin Pin límite máximo.
     */
    void setLimitSwitches(int min_pin, int max_pin);

    /**
     * @brief Ejecuta la rutina de homing.
     * @param direction Dirección hacia la que buscar el límite.
     * @param homing_speed Velocidad en RPM para homing.
     *
     * @note Requiere al menos un endstop configurado.
     */
    void home(int direction = -1, double homing_speed = 50);
   
private:
    // ---------------------------------------------------------------------
    // HILO DE CONTROL
    // ---------------------------------------------------------------------

    /**
     * @brief Hilo central que genera pasos, controla aceleración y maneja fin de carrera.
     *
     * @details
     * Se ejecuta continuamente mientras running_ sea true.
     * Maneja:
     * - generación de pulsos STEP
     * - rampas de velocidad
     * - detección de endstops
     * - homing
     */
    void stepControlThread(); 

    /**
     * @brief Genera un pulso STEP y actualiza posición.
     * @param direction Dirección del movimiento.
     */
    void generateStep(int direction);
    
    // ---------------------------------------------------------------------
    // CONVERSIONES
    // ---------------------------------------------------------------------

    /**
     * @brief Convierte RPM a pasos por segundo.
     */
    double rpmToStepsPerSec(double rpm) const;

    /**
     * @brief Convierte pasos por segundo a RPM.
     */
    double stepsPerSecToRpm(double steps_per_sec) const;
    
    // ---------------------------------------------------------------------
    // VARIABLES INTERNAS
    // ---------------------------------------------------------------------

    MotorConfig config_;                            ///< Configuración del motor
    
    std::atomic<long> current_position_;            ///< Posición actual en pasos
    std::atomic<long> target_position_;             ///< Objetivo en pasos
    std::atomic<bool> is_moving_;                   ///< Indica si está en movimiento
    std::atomic<bool> enabled_;                     ///< Indica si el motor está habilitado
    std::atomic<bool> running_;                     
    
    std::atomic<double> target_speed_steps_per_sec_; ///< Velocidad objetivo
    std::atomic<double> current_speed_steps_per_sec_;///< Velocidad actual
    std::atomic<int> current_direction_;             ///< Dirección actual

    int min_limit_pin_{-1};                         ///< Pin de límite mínimo
    int max_limit_pin_{-1};                         ///< Pin de límite máximo
    
    std::thread control_thread_;                    ///< Hilo de control
    std::mutex state_mutex_;                        ///< Mutex para operaciones críticas
    
    std::chrono::steady_clock::time_point last_step_time_; ///< Tiempo del último paso
    
    std::atomic<bool> is_homing_;                   ///< Indica si se está ejecutando homing
};

} // namespace motor

#endif
