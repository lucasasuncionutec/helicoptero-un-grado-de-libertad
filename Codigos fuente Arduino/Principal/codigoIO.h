// === codigoIO.h ===
#ifndef CODIGO_IO_H
#define CODIGO_IO_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

#define ROWS 4
#define COLS 4

// --- Variables externas de interfaz (LCD/Keypad) ---
extern char  keys[ROWS][COLS];
extern byte  rowPins[ROWS];
extern byte  colPins[COLS];
extern LiquidCrystal_I2C lcd;
extern Keypad keypad;
extern bool pid_recibido_desde_pc;
extern bool pid_recibido_desde_pc;
extern bool tss_mp_recibidos_pc;


// --- Variables compartidas con Principal.ino ---
extern float Tss;
extern float Mp;
extern bool  controlActivo;
extern float pwmAplicado;

extern float anguloActual_deg;         // valor leído desde el sensor (para display)
extern float errorActual_deg;          // error convertido a grados para mostrar
extern float angulo_equilibrio_rad;    // ángulo de equilibrio en radianes (interno)
extern float anguloReferencia_rad;     // referencia del sistema en radianes (interno)

extern float Kp;
extern float Ki;
extern float Kd;
extern float N;


extern bool  inSubMenu;


float leerAngulo();       
void  showMenu();
void  navegarMenu();
void  ingresarAnguloObjetivo();
void  ingresarTss();
void  verDatos();
bool  toggleControl();

#endif 
