import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Control de Motor DC")
        
        # Variables para la conexión serial
        self.serial_connection = None
        self.running = False
        
        # --- Frame para selección de puerto COM ---
        self.com_frame = ttk.LabelFrame(master, text="Puerto COM")
        self.com_frame.pack(padx=10, pady=5, fill="x")
        
        self.com_port_label = ttk.Label(self.com_frame, text="Selecciona puerto:")
        self.com_port_label.pack(side="left", padx=5, pady=5)
        
        self.com_port_combo = ttk.Combobox(self.com_frame, values=self.get_com_ports(), state="readonly")
        self.com_port_combo.pack(side="left", padx=5, pady=5)
        if self.com_port_combo['values']:
            self.com_port_combo.current(0)
            
        self.connect_button = ttk.Button(self.com_frame, text="Conectar", command=self.connect)
        self.connect_button.pack(side="left", padx=5, pady=5)

        # --- Frame para ingresar la posición en grados ---
        self.angle_frame = ttk.LabelFrame(master, text="Posición en grados")
        self.angle_frame.pack(padx=10, pady=5, fill="x")
        
        self.angle_label = ttk.Label(self.angle_frame, text="Ángulo (0-180):")
        self.angle_label.pack(side="left", padx=5, pady=5)
        
        self.angle_entry = ttk.Entry(self.angle_frame, width=10)
        self.angle_entry.pack(side="left", padx=5, pady=5)
        
        # --- Frame para botones de control ---
        self.button_frame = ttk.Frame(master)
        self.button_frame.pack(padx=10, pady=5)
        
        self.start_button = ttk.Button(self.button_frame, text="Iniciar", command=self.start_motor)
        self.start_button.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_button = ttk.Button(self.button_frame, text="Parar", command=self.stop_motor)
        self.stop_button.grid(row=0, column=1, padx=5, pady=5)
        
        self.reset_button = ttk.Button(self.button_frame, text="Reiniciar posición", command=self.reset_encoder)
        self.reset_button.grid(row=0, column=2, padx=5, pady=5)

        # --- Frame para mostrar la salida serial (opcional) ---
        self.output_frame = ttk.LabelFrame(master, text="Salida Serial")
        self.output_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        self.output_text = tk.Text(self.output_frame, height=10)
        self.output_text.pack(padx=5, pady=5, fill="both", expand=True)
        
    def get_com_ports(self):
        """Obtiene la lista de puertos COM disponibles."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]
    
    def connect(self):
        """Conecta al puerto COM seleccionado y comienza a leer datos."""
        selected_port = self.com_port_combo.get()
        if not selected_port:
            messagebox.showerror("Error", "No se seleccionó ningún puerto COM.")
            return
        try:
            self.serial_connection = serial.Serial(selected_port, 115200, timeout=1)
            self.running = True
            threading.Thread(target=self.read_from_serial, daemon=True).start()
            self.output_text.insert(tk.END, f"Conectado a {selected_port}\n")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar al puerto {selected_port}: {e}")
    
    def read_from_serial(self):
        """Lee datos del puerto serial y los muestra en el widget de salida."""
        while self.running and self.serial_connection and self.serial_connection.is_open:
            try:
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line:
                    self.output_text.insert(tk.END, line + "\n")
                    self.output_text.see(tk.END)
            except Exception as e:
                print("Error leyendo del puerto:", e)
    
    def start_motor(self):
        """Envía el ángulo ingresado para iniciar el movimiento del motor."""
        angle_str = self.angle_entry.get()
        try:
            angle = float(angle_str)
            if 0 <= angle <= 180:
                command = f"{angle}\n"
                self.send_command(command)
                self.output_text.insert(tk.END, f"Enviado ángulo: {angle}\n")
            else:
                messagebox.showerror("Error", "El ángulo debe estar entre 0 y 180.")
        except ValueError:
            messagebox.showerror("Error", "Ingrese un valor numérico para el ángulo.")
    
    def stop_motor(self):
        """Envía el comando de parada ('s') al microcontrolador."""
        self.send_command("s\n")
        self.output_text.insert(tk.END, "Comando de parada enviado.\n")
    
    def reset_encoder(self):
        """Envía el comando de reinicio ('r') para restablecer el encoder."""
        self.send_command("r\n")
        self.output_text.insert(tk.END, "Comando de reinicio enviado.\n")
    
    def send_command(self, command):
        """Envía un comando por el puerto serial."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.write(command.encode('utf-8'))
        else:
            messagebox.showerror("Error", "No hay conexión serial activa.")
    
    def on_closing(self):
        """Cierra la conexión serial y la ventana."""
        self.running = False
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    gui = MotorControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", gui.on_closing)
    root.mainloop()
