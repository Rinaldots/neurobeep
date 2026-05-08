import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
import bluetooth
import socket
import time
import threading
import queue
from datetime import datetime

class BluetoothGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Neurassist Bluetooth Terminal")
        self.root.geometry("800x600")
        
        # Variáveis de controle
        self.socket = None
        self.target_address = None
        self.is_connected = False
        self.receive_thread = None
        self.stop_event = threading.Event()
        self.message_queue = queue.Queue()
        
        # Configuração da interface
        self.setup_ui()
        
        # Inicia o processamento da fila de mensagens
        self.process_message_queue()
        
        # Verifica sistema Bluetooth na inicialização
        self.check_bluetooth_system()
        
    def setup_ui(self):
        """Configura a interface gráfica"""
        # Frame principal
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configuração do grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Título
        title_label = ttk.Label(main_frame, text="Neurassist Bluetooth Terminal", 
                               font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Frame de controle (esquerda)
        control_frame = ttk.LabelFrame(main_frame, text="Controle", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        # Status da conexão
        self.status_label = ttk.Label(control_frame, text="Status: Desconectado", 
                                     foreground="red")
        self.status_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))
        
        # Botões de controle
        self.scan_button = ttk.Button(control_frame, text="Escanear Dispositivos", 
                                     command=self.scan_devices)
        self.scan_button.grid(row=1, column=0, columnspan=2, pady=2, sticky=(tk.W, tk.E))
        
        # Lista de dispositivos
        ttk.Label(control_frame, text="Dispositivos:").grid(row=2, column=0, columnspan=2, 
                                                            pady=(10, 0), sticky=tk.W)
        
        self.device_listbox = tk.Listbox(control_frame, height=6)
        self.device_listbox.grid(row=3, column=0, columnspan=2, pady=2, sticky=(tk.W, tk.E))
        
        # Scrollbar para a lista
        scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=self.device_listbox.yview)
        scrollbar.grid(row=3, column=2, sticky=(tk.N, tk.S))
        self.device_listbox.configure(yscrollcommand=scrollbar.set)
        
        # Botões de conexão
        self.connect_button = ttk.Button(control_frame, text="Conectar", 
                                        command=self.connect_device)
        self.connect_button.grid(row=4, column=0, pady=5, sticky=(tk.W, tk.E))
        
        self.disconnect_button = ttk.Button(control_frame, text="Desconectar", 
                                           command=self.disconnect_device, state="disabled")
        self.disconnect_button.grid(row=4, column=1, pady=5, sticky=(tk.W, tk.E))
        
        # Frame de comunicação (direita)
        comm_frame = ttk.LabelFrame(main_frame, text="Comunicação", padding="10")
        comm_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        comm_frame.columnconfigure(0, weight=1)
        comm_frame.rowconfigure(0, weight=1)
        comm_frame.rowconfigure(2, weight=1)
        
        # Terminal de mensagens recebidas
        ttk.Label(comm_frame, text="Mensagens Recebidas:").grid(row=0, column=0, sticky=tk.W)
        
        self.received_text = scrolledtext.ScrolledText(comm_frame, height=15, width=50, 
                                                      state="disabled", wrap=tk.WORD)
        self.received_text.grid(row=1, column=0, pady=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Terminal de envio
        ttk.Label(comm_frame, text="Enviar Mensagem:").grid(row=2, column=0, sticky=tk.W, pady=(10, 0))
        
        # Frame para entrada de mensagem
        send_frame = ttk.Frame(comm_frame)
        send_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=2)
        send_frame.columnconfigure(0, weight=1)
        
        self.message_entry = ttk.Entry(send_frame, font=("Arial", 10))
        self.message_entry.grid(row=0, column=0, sticky=(tk.W, tk.E), padx=(0, 5))
        self.message_entry.bind("<Return>", self.send_message)
        
        self.send_button = ttk.Button(send_frame, text="Enviar", 
                                     command=self.send_message, state="disabled")
        self.send_button.grid(row=0, column=1)
        
        # Terminal de comandos enviados
        ttk.Label(comm_frame, text="Histórico de Envios:").grid(row=4, column=0, sticky=tk.W, pady=(10, 0))
        
        self.sent_text = scrolledtext.ScrolledText(comm_frame, height=8, width=50, 
                                                  state="disabled", wrap=tk.WORD)
        self.sent_text.grid(row=5, column=0, pady=2, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Botão para limpar terminais
        clear_button = ttk.Button(comm_frame, text="Limpar Terminais", 
                                 command=self.clear_terminals)
        clear_button.grid(row=6, column=0, pady=5, sticky=tk.W)
        
        # Label de estatísticas
        self.stats_label = ttk.Label(comm_frame, text="Enviadas: 0 | Recebidas: 0")
        self.stats_label.grid(row=7, column=0, pady=5, sticky=tk.W)
        
        # Contadores
        self.sent_count = 0
        self.received_count = 0
        
        # Frame para comandos rápidos
        quick_commands_frame = ttk.LabelFrame(control_frame, text="Comandos Rápidos", padding="5")
        quick_commands_frame.grid(row=5, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        # Botões de comandos rápidos
        commands = [
            ("Parar (P)", "P"),
            ("Andar (A)", "A"),
            ("Andar Rapido (F)", "F"),  # Comando adicional para andar rápido
            ("Girar (G)", "G"),  # Comando adicional para girar
            ("Seguir Linha (S)", "S"),
            ("Status (E)", "E")
            
        ]
        
        for i, (text, cmd) in enumerate(commands):
            btn = ttk.Button(quick_commands_frame, text=text, 
                           command=lambda c=cmd: self.send_quick_command(c))
            btn.grid(row=i//2, column=i%2, padx=2, pady=2, sticky=(tk.W, tk.E))
        
        quick_commands_frame.columnconfigure(0, weight=1)
        quick_commands_frame.columnconfigure(1, weight=1)
        
        # Configuração do redimensionamento
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        
    def check_bluetooth_system(self):
        """Verifica se o sistema Bluetooth está funcionando"""
        try:
            test_devices = bluetooth.discover_devices(duration=1, lookup_names=False)
            self.log_message("✓ Adaptador Bluetooth funcionando", "received")
            return True
        except Exception as e:
            self.log_message(f"✗ Erro no adaptador Bluetooth: {e}", "received")
            messagebox.showerror("Erro Bluetooth", 
                               f"Erro no adaptador Bluetooth: {e}\nTente executar como administrador")
            return False
    
    def scan_devices(self):
        """Escaneia dispositivos Bluetooth em thread separada"""
        self.scan_button.config(state="disabled", text="Escaneando...")
        self.device_listbox.delete(0, tk.END)
        self.log_message("🔍 Iniciando escaneamento de dispositivos...", "received")
        
        # Executa escaneamento em thread separada
        scan_thread = threading.Thread(target=self._scan_devices_thread)
        scan_thread.daemon = True
        scan_thread.start()
    
    def _scan_devices_thread(self):
        """Thread para escaneamento de dispositivos"""
        try:
            # Primeira tentativa: com nomes
            nearby_devices = bluetooth.discover_devices(duration=10, lookup_names=True)
            self.message_queue.put(("log", f"Encontrados {len(nearby_devices)} dispositivos", "received"))
            
            for addr, name in nearby_devices:
                device_info = f"{name} ({addr})"
                self.message_queue.put(("device", device_info, addr))
                
        except Exception as e:
            self.message_queue.put(("log", f"Erro no escaneamento: {e}", "received"))
            try:
                # Segunda tentativa: sem nomes
                device_addresses = bluetooth.discover_devices(duration=10, lookup_names=False)
                self.message_queue.put(("log", f"Encontrados {len(device_addresses)} endereços", "received"))
                
                for addr in device_addresses:
                    try:
                        name = bluetooth.lookup_name(addr, timeout=3)
                        if name:
                            device_info = f"{name} ({addr})"
                        else:
                            device_info = f"Dispositivo Desconhecido ({addr})"
                        self.message_queue.put(("device", device_info, addr))
                    except:
                        device_info = f"Erro ao buscar nome ({addr})"
                        self.message_queue.put(("device", device_info, addr))
                        
            except Exception as e2:
                self.message_queue.put(("log", f"Erro na segunda tentativa: {e2}", "received"))
        
        finally:
            self.message_queue.put(("scan_complete", "", ""))
    
    def connect_device(self):
        """Conecta ao dispositivo selecionado"""
        selection = self.device_listbox.curselection()
        if not selection:
            messagebox.showwarning("Aviso", "Selecione um dispositivo para conectar")
            return
        
        device_text = self.device_listbox.get(selection[0])
        # Extrai o endereço MAC do texto
        try:
            self.target_address = device_text.split('(')[1].split(')')[0]
        except:
            messagebox.showerror("Erro", "Não foi possível extrair o endereço do dispositivo")
            return
        
        self.log_message(f"🔗 Tentando conectar ao {device_text}...", "received")
        
        # Conecta em thread separada
        connect_thread = threading.Thread(target=self._connect_thread)
        connect_thread.daemon = True
        connect_thread.start()
    
    def _connect_thread(self):
        """Thread para conexão"""
        try:
            self.socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
            self.socket.settimeout(10)
            self.socket.connect((self.target_address, 1))
            self.socket.settimeout(0.5)
            
            self.message_queue.put(("connected", "", ""))
            
            # Inicia thread de recepção
            self.stop_event.clear()
            self.receive_thread = threading.Thread(target=self._receive_thread)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
        except Exception as e:
            self.message_queue.put(("connection_error", str(e), ""))
    
    def _receive_thread(self):
        """Thread para recepção de mensagens"""
        buffer = b""  # Buffer para acumular dados parciais
        while not self.stop_event.is_set() and self.socket:
            try:
                self.socket.settimeout(1.0)
                data = self.socket.recv(1024)
                if data:
                    buffer += data
                    
                    # Tenta decodificar o buffer completo
                    try:
                        # Procura por quebras de linha para processar mensagens completas
                        while b'\n' in buffer:
                            line, buffer = buffer.split(b'\n', 1)
                            if line:
                                try:
                                    message = line.decode('utf-8').strip()
                                    if message:
                                        self.message_queue.put(("receive", message, ""))
                                except UnicodeDecodeError:
                                    # Se não conseguir decodificar, tenta com latin-1
                                    try:
                                        message = line.decode('latin-1').strip()
                                        if message:
                                            self.message_queue.put(("receive", f"[RAW] {message}", ""))
                                    except:
                                        # Se ainda assim falhar, mostra em hexadecimal
                                        hex_data = ' '.join(f'{b:02x}' for b in line)
                                        self.message_queue.put(("receive", f"[HEX] {hex_data}", ""))
                        
                        # Se o buffer ficou muito grande sem quebras de linha, processa parte dele
                        if len(buffer) > 2048:
                            try:
                                message = buffer.decode('utf-8', errors='ignore').strip()
                                if message:
                                    self.message_queue.put(("receive", message, ""))
                                buffer = b""
                            except:
                                buffer = buffer[-1024:]  # Mantém apenas os últimos 1024 bytes
                                
                    except UnicodeDecodeError as e:
                        # Se há erro de decodificação, mantém os dados no buffer
                        # e tenta novamente quando mais dados chegarem
                        if len(buffer) > 2048:
                            # Se o buffer está muito grande, descarta parte dele
                            buffer = buffer[-1024:]
                        continue
                        
            except socket.timeout:
                continue
            except Exception as e:
                self.message_queue.put(("receive_error", str(e), ""))
                break
    
    def disconnect_device(self):
        """Desconecta do dispositivo"""
        self.is_connected = False
        self.stop_event.set()
        
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)
        
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        self.update_connection_status(False)
        self.log_message("🔌 Desconectado", "received")
    
    def send_message(self, event=None):
        """Envia mensagem para o dispositivo"""
        if not self.is_connected or not self.socket:
            messagebox.showwarning("Aviso", "Não conectado a nenhum dispositivo")
            return
        
        message = self.message_entry.get().strip()
        if not message:
            return
        
        try:
            # Adiciona quebra de linha se não houver
            if not message.endswith('\n'):
                message += '\n'
            
            # Envia em UTF-8 com tratamento de erro
            data_to_send = message.encode('utf-8')
            self.socket.send(data_to_send)
            
            # Log da mensagem enviada (sem a quebra de linha)
            display_message = message.rstrip('\n')
            self.log_message(f"📤 {display_message}", "sent")
            self.message_entry.delete(0, tk.END)
            
            # Atualiza contador de mensagens enviadas
            self.sent_count += 1
            self.update_stats()
            
        except Exception as e:
            self.log_message(f"⚠ Erro ao enviar: {e}", "sent")
            messagebox.showerror("Erro", f"Erro ao enviar mensagem: {e}")
            
            # Tenta reconectar se o erro for de conexão
            if "Broken pipe" in str(e) or "not connected" in str(e).lower():
                self.log_message("🔄 Tentando reconectar...", "received")
                self.disconnect_device()
                self._auto_reconnect()
    
    def _auto_reconnect(self):
        """Tenta reconectar automaticamente em thread separada"""
        if not self.target_address:
            return
        
        reconnect_thread = threading.Thread(target=self._reconnect_thread)
        reconnect_thread.daemon = True
        reconnect_thread.start()
    
    def _reconnect_thread(self):
        """Thread para reconexão automática"""
        max_attempts = 3
        for attempt in range(max_attempts):
            self.message_queue.put(("log", f"Tentativa de reconexão {attempt + 1}/{max_attempts}...", "received"))
            
            if attempt > 0:
                time.sleep(3)  # Aguarda 3 segundos entre tentativas
            
            try:
                self.socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                self.socket.settimeout(10)
                self.socket.connect((self.target_address, 1))
                self.socket.settimeout(0.5)
                
                self.message_queue.put(("connected", "", ""))
                
                # Reinicia thread de recepção
                self.stop_event.clear()
                self.receive_thread = threading.Thread(target=self._receive_thread)
                self.receive_thread.daemon = True
                self.receive_thread.start()
                
                self.message_queue.put(("log", "✓ Reconexão bem-sucedida!", "received"))
                return
                
            except Exception as e:
                self.message_queue.put(("log", f"✗ Tentativa {attempt + 1} falhou: {e}", "received"))
        
        self.message_queue.put(("log", "✗ Falha na reconexão após todas as tentativas", "received"))
    
    def process_message_queue(self):
        """Processa mensagens da fila (executado na thread principal)"""
        try:
            while True:
                msg_type, message, data = self.message_queue.get_nowait()
                
                if msg_type == "log":
                    self.log_message(message, data)
                elif msg_type == "device":
                    self.device_listbox.insert(tk.END, message)
                elif msg_type == "scan_complete":
                    self.scan_button.config(state="normal", text="Escanear Dispositivos")
                elif msg_type == "connected":
                    self.update_connection_status(True)
                    self.log_message("✓ Conectado com sucesso!", "received")
                elif msg_type == "connection_error":
                    self.log_message(f"✗ Erro na conexão: {message}", "received")
                    messagebox.showerror("Erro de Conexão", f"Não foi possível conectar: {message}")
                elif msg_type == "receive":
                    self.log_message(f"📱 {message}", "received")
                    # Atualiza contador de mensagens recebidas
                    self.received_count += 1
                    self.update_stats()
                elif msg_type == "receive_error":
                    self.log_message(f"⚠ Erro na recepção: {message}", "received")
                    
        except queue.Empty:
            pass
        
        # Reagenda para executar novamente
        self.root.after(100, self.process_message_queue)
    
    def update_connection_status(self, connected):
        """Atualiza o status da conexão na interface"""
        self.is_connected = connected
        if connected:
            self.status_label.config(text="Status: Conectado", foreground="green")
            self.connect_button.config(state="disabled")
            self.disconnect_button.config(state="normal")
            self.send_button.config(state="normal")
            self.message_entry.config(state="normal")
        else:
            self.status_label.config(text="Status: Desconectado", foreground="red")
            self.connect_button.config(state="normal")
            self.disconnect_button.config(state="disabled")
            self.send_button.config(state="disabled")
            self.message_entry.config(state="disabled")
    
    def log_message(self, message, msg_type):
        """Adiciona mensagem ao terminal apropriado"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}\n"
        
        if msg_type == "received":
            self.received_text.config(state="normal")
            self.received_text.insert(tk.END, formatted_message)
            self.received_text.config(state="disabled")
            self.received_text.see(tk.END)
                
        elif msg_type == "sent":
            self.sent_text.config(state="normal")
            self.sent_text.insert(tk.END, formatted_message)
            self.sent_text.config(state="disabled")
            self.sent_text.see(tk.END)
    
    def clear_terminals(self):
        """Limpa ambos os terminais"""
        self.received_text.config(state="normal")
        self.received_text.delete(1.0, tk.END)
        self.received_text.config(state="disabled")
        
        self.sent_text.config(state="normal")
        self.sent_text.delete(1.0, tk.END)
        self.sent_text.config(state="disabled")
        
        # Reset dos contadores
        self.sent_count = 0
        self.received_count = 0
        self.update_stats()
        
        self.log_message("🧹 Terminais limpos", "received")
    
    def update_stats(self):
        """Atualiza as estatísticas de mensagens"""
        self.stats_label.config(text=f"Enviadas: {self.sent_count} | Recebidas: {self.received_count}")
    
    def on_closing(self):
        """Cleanup quando a janela é fechada"""
        self.disconnect_device()
        self.root.destroy()

    def send_quick_command(self, command):
        """Envia comando rápido predefinido"""
        if not self.is_connected or not self.socket:
            messagebox.showwarning("Aviso", "Não conectado a nenhum dispositivo")
            return
        
        try:
            self.socket.send(command.encode('utf-8'))
            
            # Descrição do comando
            cmd_descriptions = {
                "P": "Parar motores",
                "A": "Andar para frente",
                "S": "Seguir linha",
                "E": "Status dos sensores"
            }
            
            description = cmd_descriptions.get(command, command)
            self.log_message(f"📤 {description} ({command})", "sent")
            
            # Atualiza contador
            self.sent_count += 1
            self.update_stats()
            
        except Exception as e:
            self.log_message(f"⚠ Erro ao enviar comando: {e}", "sent")
            messagebox.showerror("Erro", f"Erro ao enviar comando: {e}")
    
def main():
    root = tk.Tk()
    app = BluetoothGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
