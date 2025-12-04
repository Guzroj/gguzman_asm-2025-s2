import serial
import time
import csv

# CONFIGURACIÓN
PORT = "COM12"     # Cambia si tu Arduino usa otro COM
BAUD = 9600
CSV_FILE = "datos_puerta.csv"

print("Abriendo puerto serial...")

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Espera a que Arduino reinicie

print("Leyendo datos... (presiona CTRL+C para terminar)")

with open(CSV_FILE, "w", newline="") as file:
    writer = csv.writer(file)

    while True:
        try:
            line = ser.readline().decode("utf-8").strip()

            # Ignorar líneas vacías o texto no CSV
            if line == "" or "," not in line:
                continue

            print(line)

            # Separar por comas y guardar en el .csv
            parts = line.split(",")
            writer.writerow(parts)
            file.flush()

        except KeyboardInterrupt:
            print("\nDetenido por el usuario.")
            break
        except Exception as e:
            print("Error:", e)

ser.close()
print("Archivo guardado como:", CSV_FILE)

