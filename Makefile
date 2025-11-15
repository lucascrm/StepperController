#
# Variables de Configuración
# -------------------------

# El nombre del ejecutable final
TARGET := Test_motor

# Directorio base donde se encuentran los archivos fuente (.cpp).
SRC_DIR := .

# Directorio de salida para los archivos objeto (.o) y el ejecutable final.
BUILD_DIR := build

# Directorio(s) de inclusión (headers) personalizados.
CUSTOM_INCLUDE_DIRS := -I.

# Archivos fuente (.cpp) específicos a compilar.
# 🚨 MODIFICA ESTA LÍNEA 🚨
# Lista SÓLO los nombres de los ficheros .cpp que quieres compilar.
# Ejemplo: main.cpp configuracion.cpp modulo_a.cpp
SOURCES := Test_Motor.cpp \
	StepperController.cpp

# Opciones del compilador (ej. banderas de optimización, advertencias)
CXXFLAGS := -Wall -Wextra -std=c++17

# Bibliotecas a enlazar (pigpio y rt)
LDLIBS := -lpigpio -lrt


#
# Configuración Interna
# ----------------------

# Compilador C++
CXX := g++

# Rutas completas a los archivos fuente
FULL_SOURCES := $(addprefix $(SRC_DIR)/, $(SOURCES))

# Rutas completas a los archivos objeto (.o) en el directorio de construcción (BUILD_DIR)
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%.o, $(FULL_SOURCES))


#
# Reglas de Compilación
# ---------------------

# Regla por defecto: 'all'
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET)

# Regla para enlazar los archivos objeto y crear el ejecutable
$(BUILD_DIR)/$(TARGET): $(OBJECTS)
	@echo "🔗 Enlazando el ejecutable: $@"
	$(CXX) $(OBJECTS) -o $@ $(LDLIBS)

# Regla para compilar archivos .cpp a .o
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@echo "⚙️ Compilando: $<"
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(CUSTOM_INCLUDE_DIRS) -c $< -o $@

# Regla para crear el directorio de construcción si no existe
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)
	@echo "📦 Creado directorio de construcción: $(BUILD_DIR)"

# Regla para limpiar todos los archivos generados
.PHONY: clean
clean:
	@echo "🧹 Limpiando archivos de construcción..."
	@rm -rf $(BUILD_DIR)


# Regla para ejecutar el programa
.PHONY: run
run: all
	@echo "▶️ Ejecutando $(TARGET)..."
	@./$(BUILD_DIR)/$(TARGET)
