#!/usr/bin/env python3
"""
Interface Gráfica para Controle do Robô ESP32 via BLE
Com UUIDs específicos do ESP32
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import asyncio
from datetime import datetime
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
CHAR_UUID    = "d5593e6b-3328-493a-b3c9-9814683d8e40"

class ESP32BLEController:
    def __init__(self):
        self.client = None
        self.device_address = None
        self.loop = asyncio.get_event_loop()
        self.command_queue = asyncio.Queue()
        self.connected = asyncio.Event()
        # Inicia worker da fila de envio
        self.loop.create_task(self._command_worker())

    async def scan_devices(self, timeout=5):
        print("🔍 Escaneando dispositivos BLE...")
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            print(f"{d.address} - {d.name}")
        return devices

    async def connect(self, name_filter="MyESP32"):
        print(f"🔗 Procurando por {name_filter}...")
        devices = await BleakScanner.discover(timeout=5)
        for d in devices:
            if d.name and name_filter.lower() in d.name.lower():
                self.device_address = d.address
                print(f"Encontrado: {d.name} ({d.address})")
                break

        if not self.device_address:
            print("❌ Dispositivo não encontrado!")
            return False

        self.client = BleakClient(self.device_address, disconnected_callback=self._on_disconnect)

        try:
            await self.client.connect(timeout=10)
            if await self.client.is_connected():
                print("✅ Conectado ao ESP32 BLE")
                self.connected.set()
                return True
        except Exception as e:
            print(f"Erro ao conectar: {e}")
        return False

    async def _command_worker(self):
        """Processa fila de comandos para evitar GATT congestionado"""
        while True:
            message = await self.command_queue.get()
            try:
                await self._send_ble_async(message)
            except Exception as e:
                print(f"⚠️ Erro ao enviar '{message}': {e}")
            await asyncio.sleep(0.2)  # intervalo mínimo de segurança
            self.command_queue.task_done()

    async def _send_ble_async(self, message):
        if not self.client or not await self.client.is_connected():
            print("⚠️ Cliente BLE não está conectado, aguardando...")
            await self.connected.wait()

        data = message.encode("utf-8")
        await self.client.write_gatt_char(CHAR_UUID, data, response=True)
        print(f"📤 Enviado: {message}")

    def send_command(self, message):
        """Interface pública segura — pode ser chamada de qualquer thread"""
        asyncio.run_coroutine_threadsafe(self.command_queue.put(message), self.loop)

    async def disconnect(self):
        if self.client and await self.client.is_connected():
            await self.client.disconnect()
            print("🔌 Desconectado do ESP32")

    def _on_disconnect(self, client):
        print("⚠️ ESP32 desconectado! Tentando reconectar...")
        self.connected.clear()
        self.loop.create_task(self._auto_reconnect())

    async def _auto_reconnect(self):
        for i in range(5):
            try:
                print(f"Tentando reconexão ({i+1}/5)...")
                await self.client.connect()
                if await self.client.is_connected():
                    print("🔁 Reconectado com sucesso!")
                    self.connected.set()
                    return
            except Exception:
                await asyncio.sleep(2)
        print("❌ Falha ao reconectar.")

class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle ESP32 BLE - NeuroBeep")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Inicializa o controlador BLE
        self.controller = ESP32BLEController()
        
        # Variáveis de controle
        self.velocity_var = tk.DoubleVar(value=0.2)
        self.turn_var = tk.DoubleVar(value=0.0)
        
        # Variáveis do auto scan
        self.auto_scanning = False
        self.auto_scan_thread = None
        
        # Cria a interface
        self.create_widgets()
        
        # Thread para telemetria
        self.telemetry_running = False
        
    def create_widgets(self):
        """Cria todos os widgets da interface"""
        
        # Frame principal
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configuração de conexão
        self.create_connection_frame(main_frame)
        
        # Controles de movimento
        self.create_movement_frame(main_frame)
        
        # Controles avançados
        self.create_advanced_frame(main_frame)
        
        # Monitor de status e log
        self.create_status_frame(main_frame)
        
        # Configurar grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
    def create_connection_frame(self, parent):
        """Cria frame de conexão BLE"""
        connection_frame = ttk.LabelFrame(parent, text=f"Conexão ESP32 BLE", padding="5")
        connection_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Info dos UUIDs
        info_label = ttk.Label(connection_frame, 
                              text=f"Serviço: {SERVICE_UUID[:8]}... | Característica: {CHAR_UUID[:8]}...",
                              font=("TkDefaultFont", 8))
        info_label.grid(row=0, column=0, columnspan=6, pady=2)
        
        # Scan devices
        ttk.Button(connection_frame, text="🔍 Scan BLE", 
                  command=self.scan_devices).grid(row=1, column=0, padx=5)
        
        # Auto scan button
        self.auto_scan_btn = ttk.Button(connection_frame, text="🔄 Auto Scan", 
                                      command=self.toggle_auto_scan)
        self.auto_scan_btn.grid(row=1, column=1, padx=5)
        
        # Lista de dispositivos
        self.devices_combo = ttk.Combobox(connection_frame, width=30, state="readonly")
        self.devices_combo.grid(row=1, column=2, padx=5)
        
        # Botões conectar/desconectar
        self.connect_btn = ttk.Button(connection_frame, text="🔗 Conectar", command=self.connect)
        self.connect_btn.grid(row=1, column=3, padx=5)
        
        self.disconnect_btn = ttk.Button(connection_frame, text="🔌 Desconectar", 
                                       command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=1, column=4, padx=5)
        
        # Status da conexão
        self.connection_status = ttk.Label(connection_frame, text="❌ Desconectado", 
                                         foreground="red")
        self.connection_status.grid(row=2, column=0, columnspan=5, pady=5)
        
    def create_movement_frame(self, parent):
        """Cria frame de controles de movimento"""
        movement_frame = ttk.LabelFrame(parent, text="Controles de Movimento", padding="5")
        movement_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=(0, 5))
        
        # Botões de direção em layout de cruz
        ttk.Button(movement_frame, text="↑\nFRENTE", 
                  command=lambda: self.send_command("FRENTE")).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="←\nESQUERDA", 
                  command=lambda: self.send_command("ESQUERDA")).grid(row=1, column=0, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="⏹\nPARAR", 
                  command=lambda: self.send_command("PARAR")).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="→\nDIREITA", 
                  command=lambda: self.send_command("DIREITA")).grid(row=1, column=2, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="↓\nTRÁS", 
                  command=lambda: self.send_command("TRAS")).grid(row=2, column=1, padx=5, pady=5)
        
        # Controle de velocidade
        ttk.Label(movement_frame, text="Velocidade:").grid(row=3, column=0, sticky=tk.W, pady=10)
        velocity_scale = ttk.Scale(movement_frame, from_=-1.0, to=1.0, 
                                 variable=self.velocity_var, orient="horizontal")
        velocity_scale.grid(row=3, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.velocity_label = ttk.Label(movement_frame, text="0.20")
        self.velocity_label.grid(row=3, column=3, pady=10)
        
        velocity_scale.configure(command=self.update_velocity_label)
        
        ttk.Button(movement_frame, text="Aplicar Velocidade", 
                  command=self.set_velocity).grid(row=4, column=1, pady=5)
        
    def create_advanced_frame(self, parent):
        """Cria frame de controles avançados"""
        advanced_frame = ttk.LabelFrame(parent, text="Controles ESP32", padding="5")
        advanced_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=(5, 0))
        
        # Comandos do sistema
        ttk.Button(advanced_frame, text="START", 
                  command=lambda: self.send_command("START")).grid(row=0, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="RESET_KALMAN", 
                  command=lambda: self.send_command("RESET_KALMAN")).grid(row=1, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="CALIBRATE_IMU", 
                  command=lambda: self.send_command("CALIBRATE_IMU")).grid(row=2, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Comandos de teste
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(advanced_frame, text="🧪 Teste Conexão", 
                  command=lambda: self.send_command("TEST")).grid(row=4, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Telemetria
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=5, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.telemetry_btn = ttk.Button(advanced_frame, text="📊 Iniciar Telemetria", 
                                      command=self.toggle_telemetry)
        self.telemetry_btn.grid(row=6, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Display de telemetria
        self.telemetry_text = scrolledtext.ScrolledText(advanced_frame, width=30, height=8)
        self.telemetry_text.grid(row=7, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        advanced_frame.columnconfigure(0, weight=1)
        advanced_frame.rowconfigure(7, weight=1)
        
    def create_status_frame(self, parent):
        """Cria frame de status e log"""
        status_frame = ttk.LabelFrame(parent, text="Log de Comandos ESP32", padding="5")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(status_frame, height=8)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Botão para limpar log
        ttk.Button(status_frame, text="🗑️ Limpar Log", 
                  command=self.clear_log).grid(row=1, column=0, pady=5)
        
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        
    def scan_devices(self):
        """Escaneia dispositivos BLE"""
        self.log_message("🔍 Escaneando dispositivos BLE ESP32...")
        
        def scan_thread():
            devices = self.controller.scan_devices()
            device_list = [f"{name} ({addr})" for name, addr in devices]
            self.root.after(0, lambda: self.update_device_list(device_list))
        
        threading.Thread(target=scan_thread, daemon=True).start()
        
    def update_device_list(self, devices):
        """Atualiza lista de dispositivos"""
        self.devices_combo['values'] = devices
        if devices:
            self.devices_combo.current(0)
            self.log_message(f"✅ Encontrados {len(devices)} dispositivos BLE")
        else:
            self.log_message("❌ Nenhum dispositivo BLE encontrado")
    
    def toggle_auto_scan(self):
        """Inicia/para o escaneamento automático"""
        if not self.auto_scanning:
            self.start_auto_scan()
        else:
            self.stop_auto_scan()
    
    def start_auto_scan(self):
        """Inicia o escaneamento automático"""
        self.auto_scanning = True
        self.auto_scan_btn.config(text="🛑 Parar Auto Scan")
        self.connection_status.config(text="🔄 Auto scan ESP32 ativo...", foreground="blue")
        self.log_message("🚀 Iniciando auto scan ESP32...")
        
        # Inicia thread do auto scan
        self.auto_scan_thread = threading.Thread(target=self.auto_scan_loop, daemon=True)
        self.auto_scan_thread.start()
    
    def stop_auto_scan(self):
        """Para o escaneamento automático"""
        self.auto_scanning = False
        self.auto_scan_btn.config(text="🔄 Auto Scan")
        if not self.controller.connected:
            self.connection_status.config(text="❌ Desconectado", foreground="red")
        self.log_message("⏹️ Auto scan ESP32 parado")
    
    def auto_scan_loop(self):
        """Loop de escaneamento automático ESP32"""
        scan_count = 0
        while self.auto_scanning:
            scan_count += 1
            # Atualiza status com animação simples
            dots = "." * (scan_count % 4)
            self.root.after(0, lambda d=dots, c=scan_count: self.connection_status.config(
                text=f"🔄 Auto scan ESP32 #{c}{d}", foreground="blue"))

            self.root.after(0, lambda c=scan_count: self.log_message(f"🔍 Auto scan #{c} - Procurando ESP32test..."))

            # Escaneia dispositivos
            future = asyncio.run_coroutine_threadsafe(self.controller.scan_devices(), self.controller.loop)
            try:
                devices = future.result()  # Aguarda o resultado do scan
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Erro ao escanear dispositivos: {e}"))
                break

            # Procura especificamente por ESP32
            esp32_devices = []

            for d in devices:
                if d.name:
                    name_upper = d.name.upper()
                    # Busca por ESP32 ou termos relacionados
                    if any(term in name_upper for term in ["ESP32", "NEUROBEEP", "ESP32TEST"]):
                        esp32_devices.append((d.name, d.address))

            if esp32_devices:
                # Encontrou dispositivo ESP32 BLE
                device_list = [f"{name} ({addr})" for name, addr in esp32_devices]
                self.root.after(0, lambda dl=device_list: self.update_device_list(dl))

                # Para o auto scan e tenta conectar automaticamente
                self.root.after(0, lambda: self.on_esp32_found(esp32_devices[0]))
                break
            else:
                # Não encontrou, continua escaneando
                if devices:
                    self.root.after(0, lambda: self.log_message(f"📱 Dispositivos BLE: {[d.name for d in devices if d.name]}, mas nenhum ESP32"))
                else:
                    self.root.after(0, lambda: self.log_message("❌ Nenhum dispositivo BLE encontrado"))

                # Aguarda antes do próximo scan (3 segundos)
                for i in range(30):
                    if not self.auto_scanning:
                        return
                    time.sleep(0.1)

        # Para o auto scan quando sai do loop
        self.root.after(0, self.stop_auto_scan)
    
    def on_esp32_found(self, device_info):
        """Callback quando ESP32 BLE é encontrado"""
        name, addr = device_info
        self.stop_auto_scan()
        self.log_message(f"🎉 ESP32 BLE encontrado: {name} ({addr})")
        
        # Pergunta se quer conectar automaticamente
        if messagebox.askyesno("ESP32 BLE Encontrado!", 
                             f"ESP32 BLE encontrado: {name}\nEndereço: {addr}\n\nDeseja conectar automaticamente?"):
            # Seleciona o dispositivo na combo box
            device_display = f"{name} ({addr})"
            devices = list(self.devices_combo['values'])
            if device_display in devices:
                self.devices_combo.set(device_display)
                # Conecta automaticamente
                self.connect()
            
    def connect(self):
        """Conecta ao dispositivo selecionado"""
        if not self.devices_combo.get():
            messagebox.showwarning("Aviso", "Selecione um dispositivo ESP32 primeiro")
            return
            
        # Extrai endereço do dispositivo
        device_info = self.devices_combo.get()
        if "(" in device_info and ")" in device_info:
            address = device_info.split("(")[1].split(")")[0]
        else:
            address = "00:00:00:00:00:00"  # Simulação
            
        self.log_message(f"🔗 Conectando ao ESP32 BLE: {address}...")
        
        def connect_thread():
            success = self.controller.connect(address)
            self.root.after(0, lambda: self.on_connection_result(success))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connection_result(self, success):
        """Callback do resultado da conexão"""
        if success:
            self.connection_status.config(text="✅ ESP32 Conectado", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.log_message("🎉 Conectado ao ESP32 com sucesso!")
        else:
            self.log_message("❌ Falha na conexão com ESP32")
            messagebox.showerror("Erro", "Não foi possível conectar ao ESP32 BLE")
            
    def disconnect(self):
        """Desconecta do dispositivo"""
        self.controller.disconnect()
        self.connection_status.config(text="❌ Desconectado", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        
        # Para telemetria se estiver rodando
        if self.telemetry_running:
            self.toggle_telemetry()
            
        self.log_message("🔌 Desconectado do ESP32")
        
    def send_command(self, command):
        """Envia comando para o ESP32"""
        if not self.controller.connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        self.log_message(f"📤 Enviando comando: {command}")
        
        def send_thread():
            try:
                success = self.controller.send_command(command)
                if success:
                    self.root.after(0, lambda: self.log_message(f"✅ Comando {command} enviado com sucesso"))
                else:
                    self.root.after(0, lambda: self.log_message(f"❌ Falha ao enviar comando {command}"))
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Erro ao enviar {command}: {e}"))
        
        threading.Thread(target=send_thread, daemon=True).start()
            
    def update_velocity_label(self, value):
        """Atualiza label de velocidade"""
        self.velocity_label.config(text=f"{float(value):.2f}")
        
    def set_velocity(self):
        """Define velocidade do robô"""
        velocity = self.velocity_var.get()
        command = f"VEL:{velocity:.2f}"
        self.send_command(command)
        
    def toggle_telemetry(self):
        """Inicia/para telemetria"""
        if not self.telemetry_running:
            self.telemetry_running = True
            self.telemetry_btn.config(text="📊 Parar Telemetria")
            threading.Thread(target=self.telemetry_loop, daemon=True).start()
        else:
            self.telemetry_running = False
            self.telemetry_btn.config(text="📊 Iniciar Telemetria")
            
    def telemetry_loop(self):
        """Loop de telemetria"""
        while self.telemetry_running and self.controller.connected:
            data = self.controller.request_telemetry()
            if data:
                timestamp = datetime.now().strftime("%H:%M:%S")
                self.root.after(0, lambda: self.update_telemetry(f"[{timestamp}] {data}\n"))
            time.sleep(1)
            
    def update_telemetry(self, data):
        """Atualiza display de telemetria"""
        self.telemetry_text.insert(tk.END, data)
        self.telemetry_text.see(tk.END)
        
        # Limita o tamanho do texto
        lines = self.telemetry_text.get("1.0", tk.END).split("\n")
        if len(lines) > 50:
            self.telemetry_text.delete("1.0", "10.0")
            
    def log_message(self, message):
        """Adiciona mensagem ao log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # Imprime também no console para debug
        print(f"[{timestamp}] {message}")
        
        # Limita o tamanho do log
        lines = self.log_text.get("1.0", tk.END).split("\n")
        if len(lines) > 100:
            self.log_text.delete("1.0", "20.0")
            
    def clear_log(self):
        """Limpa o log"""
        self.log_text.delete("1.0", tk.END)


def main():
    """Função principal"""
    root = tk.Tk()
    app = RobotGUI(root)
    
    # Configura o fechamento da aplicação
    def on_closing():
        if app.auto_scanning:
            app.stop_auto_scan()
        if app.controller.connected:
            app.disconnect()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Inicia a aplicação
    root.mainloop()


if __name__ == "__main__":
    main()
