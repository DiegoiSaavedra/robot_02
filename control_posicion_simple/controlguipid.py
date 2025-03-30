import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class MotorControlGUI:
    def __init__(self, master):
        self.master = master
        master.title("Control PID doble con animacion Saavedra Labs")
        
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
        
        # --- Frame para configuración PID y MIN_PWM ---
        self.config_frame = ttk.LabelFrame(master, text="Configuración PID y MIN_PWM")
        self.config_frame.pack(padx=10, pady=5, fill="x")
        
        # Kp
        self.kp_label = ttk.Label(self.config_frame, text="Kp:")
        self.kp_label.grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.kp_entry = ttk.Entry(self.config_frame, width=10)
        self.kp_entry.grid(row=0, column=1, padx=5, pady=5)
        self.kp_entry.insert(0, "0.8")
        
        # Ki
        self.ki_label = ttk.Label(self.config_frame, text="Ki:")
        self.ki_label.grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.ki_entry = ttk.Entry(self.config_frame, width=10)
        self.ki_entry.grid(row=0, column=3, padx=5, pady=5)
        self.ki_entry.insert(0, "0.8")
        
        # Kd
        self.kd_label = ttk.Label(self.config_frame, text="Kd:")
        self.kd_label.grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.kd_entry = ttk.Entry(self.config_frame, width=10)
        self.kd_entry.grid(row=0, column=5, padx=5, pady=5)
        self.kd_entry.insert(0, "3.5")
        
        # MIN_PWM
        self.min_pwm_label = ttk.Label(self.config_frame, text="MIN_PWM:")
        self.min_pwm_label.grid(row=0, column=6, padx=5, pady=5, sticky="e")
        self.min_pwm_entry = ttk.Entry(self.config_frame, width=10)
        self.min_pwm_entry.grid(row=0, column=7, padx=5, pady=5)
        self.min_pwm_entry.insert(0, "70")
        
        self.send_config_button = ttk.Button(self.config_frame, text="Enviar Config", command=self.send_config)
        self.send_config_button.grid(row=0, column=8, padx=5, pady=5)
        
        # --- Frame para parámetros de Animación ---
        self.anim_config_frame = ttk.LabelFrame(master, text="Parámetros de Animación")
        self.anim_config_frame.pack(padx=10, pady=5, fill="x")
        
        # Command 1
        self.cmd1_label = ttk.Label(self.anim_config_frame, text="Comando 1:")
        self.cmd1_label.grid(row=0, column=0, padx=5, pady=5, sticky="e")
        self.cmd1_entry = ttk.Entry(self.anim_config_frame, width=15)
        self.cmd1_entry.grid(row=0, column=1, padx=5, pady=5)
        self.cmd1_entry.insert(0, "1:150 2:0")
        
        # Sleep (en segundos)
        self.sleep_label = ttk.Label(self.anim_config_frame, text="Sleep (s):")
        self.sleep_label.grid(row=0, column=2, padx=5, pady=5, sticky="e")
        self.sleep_entry = ttk.Entry(self.anim_config_frame, width=10)
        self.sleep_entry.grid(row=0, column=3, padx=5, pady=5)
        self.sleep_entry.insert(0, "4")
        
        # Command 2
        self.cmd2_label = ttk.Label(self.anim_config_frame, text="Comando 2:")
        self.cmd2_label.grid(row=0, column=4, padx=5, pady=5, sticky="e")
        self.cmd2_entry = ttk.Entry(self.anim_config_frame, width=15)
        self.cmd2_entry.grid(row=0, column=5, padx=5, pady=5)
        self.cmd2_entry.insert(0, "1:0 2:0")
        
        # --- Frame para el control del Motor 1 ---
        self.motor1_frame = ttk.LabelFrame(master, text="Motor 1")
        self.motor1_frame.pack(padx=10, pady=5, fill="x")
        
        self.motor1_slider = tk.Scale(self.motor1_frame, from_=0, to=180, orient="horizontal",
                                      label="Ángulo (0-180)", length=300)
        self.motor1_slider.pack(side="left", padx=5, pady=5)
        
        # --- Frame para el control del Motor 2 ---
        self.motor2_frame = ttk.LabelFrame(master, text="Motor 2")
        self.motor2_frame.pack(padx=10, pady=5, fill="x")
        
        self.motor2_slider = tk.Scale(self.motor2_frame, from_=0, to=180, orient="horizontal",
                                      label="Ángulo (0-180)", length=300)
        self.motor2_slider.pack(side="left", padx=5, pady=5)
        
        # --- Frame para botones de control ---
        self.button_frame = ttk.Frame(master)
        self.button_frame.pack(padx=10, pady=5)
        
        self.start_button = ttk.Button(self.button_frame, text="Iniciar", command=self.start_motors)
        self.start_button.grid(row=0, column=0, padx=5, pady=5)
        
        self.stop_button = ttk.Button(self.button_frame, text="Parar", command=self.stop_motors)
        self.stop_button.grid(row=0, column=1, padx=5, pady=5)
        
        self.reset_button = ttk.Button(self.button_frame, text="Reiniciar posición", command=self.reset_encoders)
        self.reset_button.grid(row=0, column=2, padx=5, pady=5)
        
        # Botón para Animación usando parámetros personalizados
        self.anim_button = ttk.Button(self.button_frame, text="Animacion1", command=self.animate_motor1)
        self.anim_button.grid(row=0, column=3, padx=5, pady=5)
        
        # --- Frame para mostrar la salida serial ---
        self.output_frame = ttk.LabelFrame(master, text="Salida Serial")
        self.output_frame.pack(padx=10, pady=5, fill="both", expand=True)
        
        # Monitor serial con fondo negro y texto verde
        self.output_text = tk.Text(self.output_frame, height=10, bg="black", fg="green")
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
        """Lee datos del puerto serial y los muestra en el monitor."""
        while self.running and self.serial_connection and self.serial_connection.is_open:
            try:
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line:
                    self.output_text.insert(tk.END, line + "\n")
                    self.output_text.see(tk.END)
            except Exception as e:
                print("Error leyendo del puerto:", e)
    
    def send_config(self):
        """Envía la configuración PID y MIN_PWM al ESP32."""
        kp = self.kp_entry.get()
        ki = self.ki_entry.get()
        kd = self.kd_entry.get()
        min_pwm = self.min_pwm_entry.get()
        pid_command = f"pid:{kp},{ki},{kd}\n"
        min_command = f"min:{min_pwm}\n"
        self.send_command(pid_command)
        self.send_command(min_command)
        self.output_text.insert(tk.END, f"Configuración enviada: {pid_command.strip()} y {min_command.strip()}\n")
    
    def animate_motor1(self):
        """
        Realiza la animación usando los parámetros personalizados:
        - Envía 'command1'
        - Espera el tiempo especificado (sleep)
        - Envía 'command2'
        """
        def task():
            # Leer los parámetros de animación desde los campos de entrada
            cmd1 = self.cmd1_entry.get().strip()
            sleep_time = self.sleep_entry.get().strip()
            cmd2 = self.cmd2_entry.get().strip()
            
            # Asegurarse de que los comandos terminen con salto de línea
            if not cmd1.endswith("\n"):
                cmd1 += "\n"
            if not cmd2.endswith("\n"):
                cmd2 += "\n"
            
            # Convertir el tiempo de espera a float
            try:
                sleep_duration = float(sleep_time)
            except ValueError:
                sleep_duration = 4.0  # Valor por defecto
            # Enviar comando 1
            self.send_command(cmd1)
            self.output_text.insert(tk.END, f"Animación: Se envió '{cmd1.strip()}'\n")
            # Esperar el tiempo especificado
            time.sleep(sleep_duration)
            # Enviar comando 2
            self.send_command(cmd2)
            self.output_text.insert(tk.END, f"Animación: Se envió '{cmd2.strip()}'\n")
        threading.Thread(target=task, daemon=True).start()
    
    def start_motors(self):
        """Envía los ángulos seleccionados para iniciar el movimiento de ambos motores."""
        angle1 = self.motor1_slider.get()
        angle2 = self.motor2_slider.get()
        command = f"1:{angle1} 2:{angle2}\n"
        self.send_command(command)
        self.output_text.insert(tk.END, f"Enviados ángulos - Motor 1: {angle1}, Motor 2: {angle2}\n")
    
    def stop_motors(self):
        """Envía el comando 's' para detener ambos motores."""
        self.send_command("s\n")
        self.output_text.insert(tk.END, "Comando de parada enviado.\n")
    
    def reset_encoders(self):
        """Envía el comando 'r' para reiniciar los encoders de ambos motores."""
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
