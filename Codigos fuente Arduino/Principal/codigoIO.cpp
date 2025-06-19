// === codigoIO.cpp =====================================================
// Manejo de LCD (I2C) + Keypad 4×4 para interfaz de usuario
// Proyecto: Helicóptero 1‑DOF – Control PIDf
// Lucas E. A. Lemos – 2025‑05‑02
// ======================================================================

#include "codigoIO.h"

// ----------------------------------------------------------------------
// Teclado y LCD
// ----------------------------------------------------------------------
char keys[ROWS][COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

byte rowPins[ROWS] = {23, 25, 27, 29};
byte colPins[COLS] = {31, 33, 35, 37};

LiquidCrystal_I2C lcd(0x27, 16, 2);
Keypad            keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ----------------------------------------------------------------------
// Menú principal
// ----------------------------------------------------------------------
const int menuLength = 4;
String menuItems[menuLength] = {
    "1.Set Ref. Angle",
    "2.Set Tss (s)",
    "3.Display Data",
    "4.Start/Stop"
};

int  currentIndex = 0;
bool inSubMenu    = false;

// ----------------------------------------------------------------------
// Utilidades de LCD
// ----------------------------------------------------------------------
static void refrescarAnguloPrincipal() {
    lcd.setCursor(5,0);
    lcd.print("        ");
    lcd.setCursor(5,0);
    lcd.print("Ang:");
    lcd.print(anguloActual_deg,2);
    lcd.write(223);
}




void showMenu() {
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Menu");
    lcd.setCursor(0,1); lcd.print(menuItems[currentIndex]);
    //refrescarAnguloPrincipal();
}

// ----------------------------------------------------------------------
// Navegación global  ►  llamar SIEMPRE en loop()
// ----------------------------------------------------------------------
void navegarMenu() {
    static unsigned long tLCD = 0;
    const unsigned long dtLCD = 250;

    if (inSubMenu) {
        switch (currentIndex) {
            case 0: ingresarAnguloObjetivo(); break;
            case 1: ingresarTss();            break;
            case 2: verDatos();               break;
        }
    }

    char key = keypad.getKey();
    if (!inSubMenu) {
        if (key=='8') { currentIndex=(currentIndex+1)%menuLength; showMenu(); }
        else if (key=='2') { currentIndex=(currentIndex-1+menuLength)%menuLength; showMenu(); }
        else if (key=='#') {
            inSubMenu = true;
            lcd.clear();
            switch (currentIndex) {
                case 0: ingresarAnguloObjetivo(); break;
                case 1: ingresarTss();            break;
                case 2: verDatos();               break;
                case 3: toggleControl();          break;
            }
        }
    }
    else if (key=='*') {
        inSubMenu = false;
        showMenu();
    }

    if ((key=='C'||key=='c')) {
        toggleControl();
        if (inSubMenu) {
            lcd.clear();
            switch (currentIndex) {
                case 0: ingresarAnguloObjetivo(); break;
                case 1: ingresarTss();            break;
                case 2: verDatos();               break;
            }
        }
    }

    if (!inSubMenu && millis()-tLCD>=dtLCD) {
        //refrescarAnguloPrincipal();
        tLCD = millis();
    }
}

// ----------------------------------------------------------------------
// Helper para redibujar buffer de ángulo
// ----------------------------------------------------------------------
static void dibujarBuffer(bool neg, const String& buf)
{
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    if (neg) lcd.print('-');
    lcd.print(buf);
}

// ----------------------------------------------------------------------
// 1) SUB‑MENÚ: Ingreso de ángulo de equilibrio
// ----------------------------------------------------------------------
void ingresarAnguloObjetivo()
{
    static String buffer   = "";
    static bool   neg      = false;
    static bool   firstMsg = true;

    if (firstMsg) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Angle (-50 to 15):");
        dibujarBuffer(neg, buffer);
        firstMsg = false;
    }

    char key = keypad.getKey();
    if (!key) return;

    if (key >= '0' && key <= '9' && buffer.length() < 3) {
        buffer += key;
        dibujarBuffer(neg, buffer);
    } else if (key == 'A') {
        neg = !neg;
        dibujarBuffer(neg, buffer);
    } else if (key == 'B' || key == 'b') {
        buffer = ""; neg = false;
        dibujarBuffer(neg, buffer);
    } else if (key == '#') {
        if (buffer.length()) {
            int ang = buffer.toInt();
            if (neg) ang = -ang;

            if (ang >= -50 && ang <= 15) {
                anguloReferencia_rad  = ang * PI / 180.0f;
                angulo_equilibrio_rad = anguloReferencia_rad;

                lcd.clear();
                lcd.setCursor(0, 0); lcd.print("Angulo set a:");
                lcd.setCursor(0, 1); lcd.print(ang);
                delay(1500);

                buffer = ""; neg = false;
                inSubMenu = false; firstMsg = true; showMenu();
            } else {
                lcd.clear();
                lcd.setCursor(0, 0); lcd.print("Fuera de rango");
                lcd.setCursor(0, 1); lcd.print("(-50 a 60)");
                delay(1500);
                buffer = ""; neg = false;
                firstMsg = true;
                ingresarAnguloObjetivo();
            }
        }
    } else if (key == '*') {
        buffer = ""; neg = false; inSubMenu = false; firstMsg = true; showMenu();
    } else if (key == 'C' || key == 'c') {
        toggleControl();
    }
}

// ----------------------------------------------------------------------
void ingresarTss()
{
    static String buffer = "";
    static bool   first  = true;

    if (first) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Tss (max 60 s):");
        lcd.setCursor(0, 1);
        first = false;
    }

    char key = keypad.getKey();
    if (!key) return;

    if (key >= '0' && key <= '9' && buffer.length() < 4) {
        buffer += key; lcd.print(key);
    } else if (key == '#') {
        if (buffer.length()) {
            float temp = buffer.toFloat();
            if (temp <= 60.0f) {
                Tss = temp;
                lcd.clear();
                lcd.setCursor(0, 0); lcd.print("Tss set a:");
                lcd.setCursor(0, 1); lcd.print(Tss);
                delay(1500);
                buffer = ""; inSubMenu = false; first = true; showMenu();
            } else {
                lcd.clear();
                lcd.setCursor(0, 0); lcd.print("Max 60 s");
                delay(1500);
                buffer = ""; lcd.setCursor(0, 1);
            }
        }
    } else if (key == '*') { buffer = ""; inSubMenu = false; first = true; showMenu(); }
    else if (key == 'C' || key == 'c') { toggleControl(); }
}

void verDatos()
{
    static int  dato  = 0;
    static bool init  = true;
    static unsigned long tLCD = 0;
    const unsigned long dtLCD = 300;

    if (init) {
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("- Display Data");
        init = false;
    }

    if (millis() - tLCD >= dtLCD) {
        lcd.setCursor(0, 1); lcd.print("                ");
        switch (dato) {
            case 0:
                lcd.setCursor(0, 1);
                lcd.print("Tss");
                if (tss_mp_recibidos_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(Tss, 1);
                lcd.print(" s");
                break;

            case 1:
                lcd.setCursor(0, 1);
                lcd.print("Mp");
                if (tss_mp_recibidos_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(Mp * 100.0f, 1);
                lcd.print(" %");
                break;

            case 2:
                lcd.setCursor(0, 1);
                lcd.print("Ang eq: ");
                lcd.print(angulo_equilibrio_rad * 180.0f / PI, 1);
                lcd.write(223);
                break;

            case 3:
                lcd.setCursor(0, 1);
                lcd.print("PWM");
                if (pwm_recibido_pc) lcd.print("*");  // ← asterisco si vino de la PC
                lcd.print(": ");
                lcd.print(pwmAplicado, 0);
                break;


            case 4:
                lcd.setCursor(0, 1);
                lcd.print("Ang act: ");
                lcd.print(anguloActual_deg, 1);
                lcd.write(223);
                break;

            case 5:
                lcd.setCursor(0, 1);
                lcd.print("Error: ");
                lcd.print(errorActual_deg, 1);
                break;

            case 6:
                lcd.setCursor(0, 1);
                lcd.print("Kp");
                if (pid_recibido_desde_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(Kp, 3);
                break;

            case 7:
                lcd.setCursor(0, 1);
                lcd.print("Ki");
                if (pid_recibido_desde_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(Ki, 3);
                break;

            case 8:
                lcd.setCursor(0, 1);
                lcd.print("Kd");
                if (pid_recibido_desde_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(Kd, 3);
                break;

            case 9:
                lcd.setCursor(0, 1);
                lcd.print("N");
                if (pid_recibido_desde_pc) lcd.print("*");
                lcd.print(": ");
                lcd.print(N, 3);
                break;
        }
        tLCD = millis();
    }

    char key = keypad.getKey();
    if (!key) return;

    if (key == '8')      { dato = (dato + 1) % 10; init = true; }
    else if (key == '2') { dato = (dato - 1 + 10) % 10; init = true; }
    else if (key == '*') { inSubMenu = false; init = true; showMenu(); }
    else if (key == 'C' || key == 'c') { toggleControl(); }
}




// ----------------------------------------------------------------------
// Alternar control/manual
// ----------------------------------------------------------------------
bool toggleControl() {
    controlActivo = !controlActivo;
    usePwmPc      = false;          // al tocar aquí, vuelvo a usar PID interno
    if (!controlActivo) pwmAplicado = 1000;

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Control:");
    lcd.setCursor(0,1); lcd.print(controlActivo ? "ENCENDIDO" : "APAGADO");
    delay(1000);

    if (!inSubMenu) showMenu();
    return controlActivo;
}
