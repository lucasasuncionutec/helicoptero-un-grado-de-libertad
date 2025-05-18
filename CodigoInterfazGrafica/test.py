import serial
import time

# === CONFIGURACIÓN DEL PUERTO SERIAL ===
puerto = 'COM5'      # Cambiar según tu sistema
baudrate = 9600
timeout = 1           # En segundos, no milisegundos

try:
    arduino = serial.Serial(port=puerto, baudrate=baudrate, timeout=timeout)
    time.sleep(2)  # Esperar a que se estabilice la conexión
    print(f"✅ Conectado a {puerto} a {baudrate} bps.")
except Exception as e:
    print(f"❌ No se pudo abrir el puerto {puerto}. Error: {e}")
    exit()

def comunicacion_serial():
    try:
        while True:
            print("\nOpciones:")
            print("1. Enviar nuevo Tss y Mp")
            print("2. Leer datos del Arduino")
            print("3. Salir")
            opcion = input("Elegí una opción: ")

            if opcion == "1":
                try:
                    Tss = float(input("Ingresá el nuevo valor de Tss: "))
                    Mp  = float(input("Ingresá el nuevo valor de Mp: "))
                    comando = f"{Tss:.3f},{Mp:.3f}\n"
                    arduino.write(comando.encode())
                    print(f"📤 Enviado: {comando.strip()}")
                except ValueError:
                    print("⚠️ Error: Tss y Mp deben ser números válidos.")

            elif opcion == "2":
                print("📥 Esperando datos del Arduino (Ctrl+C para cancelar)...")
                try:
                    while True:
                        if arduino.in_waiting:
                            linea = arduino.readline().decode(errors='ignore').strip()
                            if linea:
                                print("Arduino:", linea)
                        else:
                            time.sleep(0.1)
                except KeyboardInterrupt:
                    print("⏹️ Lectura interrumpida por el usuario.")

            elif opcion == "3":
                print("🔌 Cerrando conexión...")
                break

            else:
                print("❌ Opción inválida. Elegí 1, 2 o 3.")

    except KeyboardInterrupt:
        print("\n⏹️ Interrumpido por el usuario.")

    finally:
        if arduino.is_open:
            arduino.close()
            print("🔒 Puerto cerrado.")

comunicacion_serial()
