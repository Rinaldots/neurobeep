#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import bluetooth
import socket
import threading
import time
from datetime import datetime

class BluetoothGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Neurassist Bluetooth Controller")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Variáveis de estado
        self.socket = None
        self.connected = False
        self.target_address = None
        self.auto_reconnect_enabled = True
        self.receiving_thread = None
        self.stop_receiving = False
        
        # Configurar estilo
        self.setup_style()
        
        # Criar interface
        self.create_widgets()
        
        # Iniciar verificação de dispositivos
        self.scan_devices()
        
    def setup_style(self):
        """Configura o estilo da interface"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # Cores personalizadas
        style.configure('Title.TLabel', 
                       background='#2b2b2b', 
                       foreground='#ffffff', 
                       font=('Arial', 16, 'bold'))
        
        style.configure('Status.TLabel', 
                       background='#2b2b2b', 
                       foreground='#00ff00', 
                       font=('Arial', 10, 'bold'))
        
        style.configure('Error.TLabel', 
                       background='#2b2b2b', 
                       foreground='#ff4444', 
                       font=('Arial', 10, 'bold'))
        
    def create_widgets(self):
        """Cria todos os widgets da interface"""
        
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Título
        title_label = ttk.Label(main_frame, text="🤖 Neurassist Bluetooth Controller", 
                               style='Title.TLabel')
        title_label.pack(pady=(0, 10))
        
        # Frame de conexão
        connection_frame = ttk.LabelFrame(main_frame, text="🔗 Conexão Bluetooth")
        connection_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Dispositivos encontrados
        ttk.Label(connection_frame, text="Dispositivos disponíveis:").pack(anchor=tk.W, padx=5, pady=2)
        
        device_frame = ttk.Frame(connection_frame)
        device_frame.pack(fill=tk.X, padx=5, pady=2)
        
        self.device_combo = ttk.Combobox(device_frame, state="readonly", width=50)
        self.device_combo.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        self.scan_button = ttk.Button(device_frame, text="🔍 Escanear", 
                                     command=self.scan_devices, width=12)
        self.scan_button.pack(side=tk.RIGHT, padx=(5, 0))
        
        # Botões de conexão
        button_frame = ttk.Frame(connection_frame)
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.connect_button = ttk.Button(button_frame, text="🔌 Conectar", 
                                        command=self.connect_device)
        self.connect_button.pack(side=tk.LEFT, padx=(0, 5))
        
        self.disconnect_button = ttk.Button(button_frame, text="🔌 Desconectar", 
                                           command=self.disconnect_device, state=tk.DISABLED)
        self.disconnect_button.pack(side=tk.LEFT, padx=(0, 5))
        
        # Status da conexão
        self.status_label = ttk.Label(connection_frame, text="❌ Desconectado", 
                                     style='Error.TLabel')
        self.status_label.pack(anchor=tk.W, padx=5, pady=2)
        
        # Checkbox de reconexão automática
        self.auto_reconnect_var = tk.BooleanVar(value=True)
        auto_reconnect_check = ttk.Checkbox(connection_frame, 
                                           text="🔄 Reconexão automática", 
                                           variable=self.auto_reconnect_var)
        auto_reconnect_check.pack(anchor=tk.W, padx=5, pady=2)
        
        # Frame de comunicação
        comm_frame = ttk.LabelFrame(main_frame, text="💬 Comunicação")
        comm_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        # Área de log/feedback
        log_frame = ttk.Frame(comm_frame)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        ttk.Label(log_frame, text="📨 Log de Comunicação:").pack(anchor=tk.W)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, 
                                                 height=15, 
                                                 bg='#1e1e1e', 
                                                 fg='#ffffff', 
                                                 font=('Consolas', 10))
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(2, 0))
        
        # Frame de envio
        send_frame = ttk.Frame(comm_frame)
        send_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(send_frame, text="📤 Enviar comando:").pack(anchor=tk.W)
        
        input_frame = ttk.Frame(send_frame)
        input_frame.pack(fill=tk.X, pady=(2, 0))
        
        self.message_entry = ttk.Entry(input_frame, font=('Arial', 11))
        self.message_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.message_entry.bind('<Return>', self.send_message)
        
        self.send_button = ttk.Button(input_frame, text="📤 Enviar", 
                                     command=self.send_message, state=tk.DISABLED)
        self.send_button.pack(side=tk.RIGHT, padx=(5, 0))
        
        # Frame de comandos rápidos
        quick_frame = ttk.LabelFrame(main_frame, text="⚡ Comandos Rápidos")
        quick_frame.pack(fill=tk.X)
        
        buttons_frame = ttk.Frame(quick_frame)
        buttons_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Comandos rápidos do robô
        quick_commands = [
            ("🛑 Parar", "P"),
            ("⬆️ Frente", "A"), 
            ("📏 Seguir Linha", "S"),
            ("📍 Andar 15cm", "1"),
            ("🔄 Status", "status")
        ]
        
        for i, (text, command) in enumerate(quick_commands):
            btn = ttk.Button(buttons_frame, text=text, 
                           command=lambda cmd=command: self.send_quick_command(cmd))
            btn.grid(row=0, column=i, padx=2, sticky=tk.EW)
            buttons_frame.grid_columnconfigure(i, weight=1)
        
        # Adicionar timestamp ao log inicial
        self.add_log("🚀 Interface iniciada", "SYSTEM")
        
    def add_log(self, message, msg_type="INFO"):
        """Adiciona mensagem ao log com timestamp e tipo"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Cores por tipo
        colors = {
            "SYSTEM": "#00ffff",
            "SEND": "#00ff00", 
            "RECV": "#ffff00",
            "ERROR": "#ff4444",
            "INFO": "#ffffff"
        }
        
        color = colors.get(msg_type, "#ffffff")
        
        # Inserir no final
        self.log_text.insert(tk.END, f"[{timestamp}] [{msg_type}] {message}\n")
        
        # Aplicar cor à linha atual
        line_start = self.log_text.index("end-2c linestart")
        line_end = self.log_text.index("end-2c lineend")
        
        tag_name = f"color_{msg_type}_{timestamp}"
        self.log_text.tag_add(tag_name, line_start, line_end)
        self.log_text.tag_config(tag_name, foreground=color)
        
        # Auto-scroll para o final
        self.log_text.see(tk.END)
        
        # Limitar o número de linhas (manter apenas as últimas 1000)
        lines = self.log_text.get("1.0", tk.END).count('\n')
        if lines > 1000:
            self.log_text.delete("1.0", "100.0")
    
    def scan_devices(self):
        """Escaneia dispositivos Bluetooth disponíveis"""
        self.add_log("🔍 Escaneando dispositivos Bluetooth...", "SYSTEM")
        self.scan_button.configure(text="⏳ Escaneando...", state=tk.DISABLED)
        
        def scan_thread():
            try:
                # Descobrir dispositivos
                nearby_devices = []
                
                try:
                    devices = bluetooth.discover_devices(duration=8, lookup_names=True)
                    nearby_devices = devices
                except:
                    # Fallback: descobrir sem nomes e buscar nomes depois
                    addresses = bluetooth.discover_devices(duration=8, lookup_names=False)
                    for addr in addresses:
                        try:
                            name = bluetooth.lookup_name(addr, timeout=3)
                            nearby_devices.append((addr, name or "Desconhecido"))
                        except:
                            nearby_devices.append((addr, "Erro"))
                
                # Atualizar interface na thread principal
                self.root.after(0, self.update_device_list, nearby_devices)
                
            except Exception as e:
                self.root.after(0, lambda: self.add_log(f"❌ Erro no escaneamento: {e}", "ERROR"))
            finally:
                self.root.after(0, lambda: self.scan_button.configure(text="🔍 Escanear", state=tk.NORMAL))
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def update_device_list(self, devices):
        """Atualiza a lista de dispositivos no combobox"""
        device_list = []
        esp_devices = []
        
        for addr, name in devices:
            device_str = f"{name} ({addr})"
            device_list.append(device_str)
            
            # Priorizar dispositivos ESP
            if name and ("esp" in name.lower() or "neuro" in name.lower()):
                esp_devices.append(device_str)
        
        # Colocar dispositivos ESP no topo
        final_list = esp_devices + [d for d in device_list if d not in esp_devices]
        
        self.device_combo['values'] = final_list
        
        if final_list:
            self.device_combo.current(0)  # Selecionar o primeiro (provavelmente ESP)
            self.add_log(f"✅ Encontrados {len(final_list)} dispositivos", "SYSTEM")
        else:
            self.add_log("⚠️ Nenhum dispositivo encontrado", "SYSTEM")
    
    def connect_device(self):
        """Conecta ao dispositivo selecionado"""
        if not self.device_combo.get():
            messagebox.showwarning("Aviso", "Selecione um dispositivo primeiro!")
            return
        
        # Extrair endereço MAC do texto selecionado
        device_text = self.device_combo.get()
        try:
            self.target_address = device_text.split('(')[1].split(')')[0]
        except:
            messagebox.showerror("Erro", "Formato de dispositivo inválido!")
            return
        
        self.add_log(f"🔌 Conectando ao {device_text}...", "SYSTEM")
        
        def connect_thread():
            success = self.establish_connection()
            self.root.after(0, lambda: self.on_connection_result(success))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def establish_connection(self):
        """Estabelece conexão Bluetooth"""
        try:
            self.socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.socket.settimeout(10)
            self.socket.connect((self.target_address, 1))
            self.socket.settimeout(0.5)
            return True
        except Exception as e:
            self.add_log(f"❌ Erro na conexão: {e}", "ERROR")
            return False
    
    def on_connection_result(self, success):
        """Callback para resultado da conexão"""
        if success:
            self.connected = True
            self.status_label.configure(text="✅ Conectado", style='Status.TLabel')
            self.connect_button.configure(state=tk.DISABLED)
            self.disconnect_button.configure(state=tk.NORMAL)
            self.send_button.configure(state=tk.NORMAL)
            self.add_log("✅ Conexão estabelecida com sucesso!", "SYSTEM")
            
            # Iniciar thread de recebimento
            self.start_receiving()
        else:
            self.connected = False
            self.status_label.configure(text="❌ Falha na conexão", style='Error.TLabel')
            self.add_log("❌ Falha ao conectar", "ERROR")
    
    def disconnect_device(self):
        """Desconecta do dispositivo"""
        self.connected = False
        self.stop_receiving = True
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        self.status_label.configure(text="❌ Desconectado", style='Error.TLabel')
        self.connect_button.configure(state=tk.NORMAL)
        self.disconnect_button.configure(state=tk.DISABLED)
        self.send_button.configure(state=tk.DISABLED)
        self.add_log("🔌 Desconectado", "SYSTEM")
    
    def start_receiving(self):
        """Inicia thread para receber dados"""
        self.stop_receiving = False
        
        def receive_thread():
            while not self.stop_receiving and self.connected:
                try:
                    if self.socket:
                        data = self.socket.recv(1024)
                        if data:
                            message = data.decode('utf-8').strip()
                            self.root.after(0, lambda msg=message: self.add_log(f"📥 {msg}", "RECV"))
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.connected:  # Só mostra erro se ainda deveria estar conectado
                        self.root.after(0, lambda: self.add_log(f"❌ Erro ao receber: {e}", "ERROR"))
                        
                        # Tentar reconectar se habilitado
                        if self.auto_reconnect_var.get():
                            self.root.after(0, self.attempt_reconnect)
                    break
        
        self.receiving_thread = threading.Thread(target=receive_thread, daemon=True)
        self.receiving_thread.start()
    
    def attempt_reconnect(self):
        """Tenta reconectar automaticamente"""
        if not self.connected or not self.auto_reconnect_var.get():
            return
        
        self.add_log("🔄 Tentando reconectar...", "SYSTEM")
        
        def reconnect_thread():
            time.sleep(2)  # Aguardar um pouco
            if self.establish_connection():
                self.root.after(0, lambda: self.add_log("✅ Reconectado com sucesso!", "SYSTEM"))
                self.root.after(0, self.start_receiving)
            else:
                self.root.after(0, lambda: self.add_log("❌ Falha na reconexão", "ERROR"))
                self.root.after(0, self.disconnect_device)
        
        threading.Thread(target=reconnect_thread, daemon=True).start()
    
    def send_message(self, event=None):
        """Envia mensagem digitada"""
        message = self.message_entry.get().strip()
        if not message:
            return
        
        if not self.connected or not self.socket:
            messagebox.showwarning("Aviso", "Não conectado ao dispositivo!")
            return
        
        try:
            self.socket.send(message.encode('utf-8'))
            self.add_log(f"📤 {message}", "SEND")
            self.message_entry.delete(0, tk.END)
        except Exception as e:
            self.add_log(f"❌ Erro ao enviar: {e}", "ERROR")
            if self.auto_reconnect_var.get():
                self.attempt_reconnect()
    
    def send_quick_command(self, command):
        """Envia comando rápido"""
        if not self.connected or not self.socket:
            messagebox.showwarning("Aviso", "Não conectado ao dispositivo!")
            return
        
        try:
            self.socket.send(command.encode('utf-8'))
            self.add_log(f"⚡ {command}", "SEND")
        except Exception as e:
            self.add_log(f"❌ Erro ao enviar comando: {e}", "ERROR")
            if self.auto_reconnect_var.get():
                self.attempt_reconnect()
    
    def on_closing(self):
        """Callback para fechamento da janela"""
        self.disconnect_device()
        self.root.destroy()

def main():
    root = tk.Tk()
    app = BluetoothGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
