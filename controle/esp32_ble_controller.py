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
import re

SERVICE_UUID = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
CHAR_UUID    = "d5593e6b-3328-493a-b3c9-9814683d8e40"

class TelemetryParser:
    """Parser para dados de telemetria do ESP32"""
    
    @staticmethod
    def parse(data_str):
        """Parse string de telemetria para dicionário"""
        result = {}
        
        # Pattern: NOME:valor
        pattern = r'([A-Z_]+):([-+]?[0-9]*\.?[0-9]+)'
        matches = re.findall(pattern, data_str)
        
        for key, value in matches:
            try:
                # Tenta converter para float
                if '.' in value:
                    result[key] = float(value)
                else:
                    result[key] = int(value)
            except ValueError:
                result[key] = value
        
        return result

class TelemetryDisplay:
    """Display de telemetria com barras de progresso e valores"""
    
    def __init__(self, parent_frame):
        self.parent = parent_frame
        self.labels = {}
        self.progressbars = {}
        self.value_labels = {}
        
        # Canvas com scrollbar
        canvas = tk.Canvas(parent_frame, bg='#2b2b2b')
        scrollbar = ttk.Scrollbar(parent_frame, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Grupos de dados
        self.create_group("🚗 Encoders", [
            ("ENC_L", "Encoder Esquerdo", 0, 1000),
            ("ENC_R", "Encoder Direito", 0, 1000)
        ])
        
        self.create_group("🏎️ Velocidades (m/s)", [
            ("VEL_L", "Velocidade Esquerda", -0.5, 0.5),
            ("VEL_R", "Velocidade Direita", -0.5, 0.5),
            ("TGT_L", "Target Esquerda", -0.5, 0.5),
            ("TGT_R", "Target Direita", -0.5, 0.5)
        ])
        
        self.create_group("⚡ PWM (0-255)", [
            ("PWM_L", "PWM Esquerdo", 0, 255),
            ("PWM_R", "PWM Direito", 0, 255),
            ("GAIN_L", "Gain Esquerdo", 0, 255),
            ("GAIN_R", "Gain Direito", 0, 255)
        ])
        
        self.create_group("📍 Odometria", [
            ("ODOM_X", "Posição X (m)", -5, 5),
            ("ODOM_Y", "Posição Y (m)", -5, 5),
            ("ODOM_TH", "Ângulo θ (rad)", -3.14, 3.14),
            ("ODOM_VX", "Velocidade X", -1, 1),
            ("ODOM_VY", "Velocidade Y", -1, 1),
            ("ODOM_W", "Velocidade Angular", -2, 2)
        ])
        
        self.create_group("📡 IMU Acelerômetro (m/s²)", [
            ("IMU_AX", "Aceleração X", -20, 20),
            ("IMU_AY", "Aceleração Y", -20, 20),
            ("IMU_AZ", "Aceleração Z", -20, 20)
        ])
        
        self.create_group("🌀 IMU Giroscópio (rad/s)", [
            ("IMU_GX", "Giro X", -5, 5),
            ("IMU_GY", "Giro Y", -5, 5),
            ("IMU_GZ", "Giro Z", -5, 5)
        ])
        
        self.create_group("📏 Sensor de Linha", [
            ("LINE_POS", "Posição", -1000, 1000),
            ("LINE_DIST", "Distância", -1000, 1000)
        ])
        
        self.create_group("🌍 GPS", [
            ("GPS_LAT", "Latitude", -90, 90),
            ("GPS_LNG", "Longitude", -180, 180),
            ("GPS_ALT", "Altitude (m)", 0, 1000),
            ("GPS_SPD", "Velocidade", 0, 100),
            ("GPS_VAL", "Válido", 0, 1)
        ])
        
    def create_group(self, title, fields):
        """Cria um grupo de displays com título"""
        group_frame = ttk.LabelFrame(self.scrollable_frame, text=title, padding=10)
        group_frame.pack(fill=tk.X, padx=5, pady=5)
        
        for i, (key, label, min_val, max_val) in enumerate(fields):
            # Frame para cada item
            item_frame = ttk.Frame(group_frame)
            item_frame.pack(fill=tk.X, pady=2)
            
            # Label do campo
            lbl = ttk.Label(item_frame, text=label, width=20, anchor='w')
            lbl.pack(side=tk.LEFT, padx=5)
            
            # Progressbar
            pb = ttk.Progressbar(item_frame, length=200, mode='determinate')
            pb.pack(side=tk.LEFT, padx=5)
            pb['maximum'] = 100
            
            # Label do valor
            val_lbl = ttk.Label(item_frame, text="0.000", width=12, anchor='e',
                               font=('Consolas', 10, 'bold'))
            val_lbl.pack(side=tk.LEFT, padx=5)
            
            # Armazena referências
            self.labels[key] = lbl
            self.progressbars[key] = pb
            self.value_labels[key] = val_lbl
            
            # Armazena limites para cálculo de porcentagem
            pb.min_val = min_val
            pb.max_val = max_val
    
    def update_value(self, key, value):
        """Atualiza um valor específico"""
        if key in self.progressbars:
            pb = self.progressbars[key]
            val_lbl = self.value_labels[key]
            
            # Atualiza label do valor
            if isinstance(value, float):
                val_lbl['text'] = f"{value:.3f}"
            else:
                val_lbl['text'] = f"{value}"
            
            # Calcula porcentagem para progressbar
            min_val = pb.min_val
            max_val = pb.max_val
            range_val = max_val - min_val
            
            if range_val != 0:
                percentage = ((value - min_val) / range_val) * 100
                percentage = max(0, min(100, percentage))  # Limita entre 0-100
                pb['value'] = percentage
            else:
                pb['value'] = 50
    
    def update_data(self, telemetry_dict):
        """Atualiza todos os valores de telemetria"""
        for key, value in telemetry_dict.items():
            self.update_value(key, value)
    
    def clear_data(self):
        """Limpa todos os valores"""
        for key in self.progressbars:
            self.progressbars[key]['value'] = 0
            self.value_labels[key]['text'] = "0.000"

class ESP32BLEController:
    def __init__(self):
        self.client = None
        self.device_address = None
        self._reconnect_in_progress = False
        self._manual_disconnect = False
        
        # Buffer para telemetria recebida
        self.telemetry_data = ""
        self.telemetry_callback = None

        # Event loop dedicado em thread separada
        self.loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        # Placeholders inicializados no bootstrap assíncrono
        self.command_queue = None
        self.connected = None

        # Inicializa estruturas assíncronas dentro do loop dedicado
        asyncio.run_coroutine_threadsafe(self._bootstrap_async(), self.loop)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _bootstrap_async(self):
        # Criadas dentro do loop para evitar problemas de binding
        self.command_queue = asyncio.Queue()
        self.connected = asyncio.Event()
        # Inicia worker de envio
        self.loop.create_task(self._command_worker())

    async def scan_devices(self, timeout=5):
        print("🔍 Escaneando dispositivos BLE...")
        devices = await BleakScanner.discover(timeout=timeout)
        for d in devices:
            print(f"{d.address} - {d.name}")
        return devices

    async def connect_by_address(self, address: str):
        """Conecta diretamente a um endereço BLE."""
        self.device_address = address
        self.client = BleakClient(self.device_address, disconnected_callback=self._on_disconnect)
        try:
            await self.client.connect(timeout=10)
            if self.client.is_connected:
                print("✅ Conectado ao ESP32 BLE")
                
                # Ativa notificações para receber telemetria
                await self.client.start_notify(CHAR_UUID, self._notification_handler)
                print("📡 Notificações BLE ativadas")
                
                if self.connected:
                    self.connected.set()
                return True
        except Exception as e:
            print(f"Erro ao conectar: {e}")
        return False

    def _notification_handler(self, sender, data):
        """Handler para notificações BLE recebidas do ESP32"""
        try:
            received = data.decode("utf-8")
            self.telemetry_data = received
            #print(f"📥 Telemetria recebida: {received}")
            
            # Chama callback se existir (para atualizar GUI)
            if self.telemetry_callback:
                self.telemetry_callback(received)
        except Exception as e:
            print(f"Erro ao processar notificação: {e}")

    async def _command_worker(self):
        """Processa fila de comandos para evitar GATT congestionado"""
        while True:
            message = await self.command_queue.get()
            try:
                await self._send_ble_async(message)
            except Exception as e:
                print(f"⚠️ Erro ao enviar '{message}': {e}")
            await asyncio.sleep(0.2)
            self.command_queue.task_done()

    async def _send_ble_async(self, message):
        if not self.client or not self.client.is_connected:
            print("⚠️ Cliente BLE não está conectado, aguardando...")
            if self.connected:
                await self.connected.wait()

        data = message.encode("utf-8")
        await self.client.write_gatt_char(CHAR_UUID, data, response=True)
        #print(f"📤 Enviado: {message}")

    def send_command(self, message):
        """Interface pública segura — pode ser chamada de qualquer thread"""
        asyncio.run_coroutine_threadsafe(self.command_queue.put(message), self.loop)
        return True

    async def disconnect(self):
        self._manual_disconnect = True  # Sinaliza desconexão manual
        if self.client:
            try:
                if self.client.is_connected:
                    await self.client.disconnect()
                    print("🔌 Desconectado do ESP32")
            finally:
                if self.connected:
                    self.connected.clear()

    def _on_disconnect(self, client):
        if self._manual_disconnect:
            print("🔌 Desconexão manual realizada")
            self._manual_disconnect = False
            return
            
        print("⚠️ ESP32 desconectado inesperadamente!")
        if self.connected:
            self.connected.clear()
            
        # Evita múltiplas tentativas de reconexão simultâneas
        if not self._reconnect_in_progress:
            self.loop.create_task(self._auto_reconnect())

    async def _auto_reconnect(self):
        if self._reconnect_in_progress:
            return
            
        self._reconnect_in_progress = True
        print("🔄 Iniciando processo de reconexão...")
        
        try:
            # Aguarda um pouco antes de tentar reconectar
            await asyncio.sleep(3)
            
            for i in range(3):  # Reduzido de 5 para 3 tentativas
                if self._manual_disconnect:  # Se desconexão manual, para o processo
                    break
                    
                try:
                    print(f"Tentando reconexão ({i+1}/3)...")
                    
                    # Recria o cliente para evitar problemas de estado
                    self.client = BleakClient(self.device_address, disconnected_callback=self._on_disconnect)
                    await self.client.connect(timeout=15)
                    
                    if self.client.is_connected:
                        print("🔁 Reconectado com sucesso!")
                        if self.connected:
                            self.connected.set()
                        return
                        
                except Exception as e:
                    print(f"Erro na tentativa {i+1}: {e}")
                    
                await asyncio.sleep(5)  # Aumenta delay entre tentativas
                
            print("❌ Falha ao reconectar após 3 tentativas.")
            
        finally:
            self._reconnect_in_progress = False

    def is_connected(self) -> bool:
        """Retorna estado de conexão de forma thread-safe."""
        try:
            # Se não houver client, não está conectado
            if self.client is None:
                return False
            fut = asyncio.run_coroutine_threadsafe(self.client.is_connected, self.loop)
            return bool(fut.result(timeout=2))
        except Exception:
            return False

    def request_telemetry(self):
        """Solicita telemetria do ESP32 enviando comando RQS:"""
        try:
            self.send_command("RQS:")
            return self.telemetry_data  # Retorna último dado recebido
        except Exception as e:
            print(f"Erro ao solicitar telemetria: {e}")
            return ""
    
    def set_telemetry_callback(self, callback):
        """Define callback para quando telemetria for recebida"""
        self.telemetry_callback = callback

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
        self.telemetry_hz = tk.IntVar(value=1)  # Taxa de telemetria em Hz
        
        # Variáveis do auto scan
        self.auto_scanning = False
        self.auto_scan_thread = None
        
        # Cria a interface
        self.create_widgets()
        
        # Thread para telemetria
        self.telemetry_running = False
        
        # Configura callback de telemetria
        self.controller.set_telemetry_callback(self.on_telemetry_received)
        
    def create_widgets(self):
        """Cria todos os widgets da interface"""
        
        # Notebook (abas)
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Aba 1: Controles
        control_tab = ttk.Frame(notebook)
        notebook.add(control_tab, text="🎮 Controles")
        
        # Frame principal da aba de controles
        main_frame = ttk.Frame(control_tab, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configuração de conexão
        self.create_connection_frame(main_frame)
        
        # Controles de movimento
        self.create_movement_frame(main_frame)
        
        # Controles avançados
        self.create_advanced_frame(main_frame)
        
        # Monitor de status e log
        self.create_status_frame(main_frame)
        
        # Configurar grid weights da aba de controles
        control_tab.columnconfigure(0, weight=1)
        control_tab.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # Aba 2: Telemetria
        telemetry_tab = ttk.Frame(notebook)
        notebook.add(telemetry_tab, text="📊 Telemetria")
        
        # Toolbar para telemetria
        toolbar_frame = ttk.Frame(telemetry_tab)
        toolbar_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(toolbar_frame, text="️ Limpar", 
                  command=self.clear_telemetry_display).pack(side=tk.LEFT, padx=5)
        
        # Frame para display de telemetria
        display_frame = ttk.Frame(telemetry_tab)
        display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Cria display de telemetria
        print("📊 Criando display de telemetria...")
        self.telemetry_display = TelemetryDisplay(display_frame)
        print("✅ Display de telemetria ativado!")
        
        # Configurar grid weights principal
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
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
                  command=lambda: self.send_command("CMD:VEL:0.3 0.3")).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="←\nESQUERDA", 
                  command=lambda: self.send_command("CMD:VEL:-0.2 0.2")).grid(row=1, column=0, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="⏹\nPARAR", 
                  command=lambda: self.send_command("CMD:VEL:0.0 0.0")).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="→\nDIREITA", 
                  command=lambda: self.send_command("CMD:VEL:0.2 -0.2")).grid(row=1, column=2, padx=5, pady=5)

        ttk.Button(movement_frame, text="↓\nTRÁS", 
                  command=lambda: self.send_command("CMD:VEL:-0.3 -0.3")).grid(row=2, column=1, padx=5, pady=5)

        # Controle de velocidade
        ttk.Label(movement_frame, text="Velocidade:").grid(row=3, column=0, sticky=tk.W, pady=10)
        velocity_scale = ttk.Scale(movement_frame, from_=-1.0, to=1.0, 
                                 variable=self.velocity_var, orient="horizontal")
        velocity_scale.grid(row=3, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.velocity_label = ttk.Label(movement_frame, text="0.30")
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
                  command=lambda: self.send_command("CMD:START")).grid(row=0, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="RESET_KALMAN", 
                  command=lambda: self.send_command("CMD:RESET_KALMAN")).grid(row=1, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="CALIBRATE_IMU", 
                  command=lambda: self.send_command("CMD:CALIBRATE_IMU")).grid(row=2, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Comandos de teste
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(advanced_frame, text="🧪 Teste Conexão", 
                  command=lambda: self.send_command("CMD:TEST")).grid(row=4, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Telemetria
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=5, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # Controle de taxa de atualização
        hz_frame = ttk.Frame(advanced_frame)
        hz_frame.grid(row=6, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Label(hz_frame, text="Taxa:").pack(side=tk.LEFT, padx=(0, 5))
        hz_spinbox = ttk.Spinbox(hz_frame, from_=1, to=10, width=5, 
                                textvariable=self.telemetry_hz, state="readonly")
        hz_spinbox.pack(side=tk.LEFT, padx=(0, 5))
        ttk.Label(hz_frame, text="Hz").pack(side=tk.LEFT)
        
        self.telemetry_btn = ttk.Button(advanced_frame, text="📊 Iniciar Telemetria", 
                                      command=self.toggle_telemetry)
        self.telemetry_btn.grid(row=7, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        # Display de telemetria
        self.telemetry_text = scrolledtext.ScrolledText(advanced_frame, width=30, height=8)
        self.telemetry_text.grid(row=8, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        advanced_frame.columnconfigure(0, weight=1)
        advanced_frame.rowconfigure(8, weight=1)
        
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
            try:
                future = asyncio.run_coroutine_threadsafe(self.controller.scan_devices(), self.controller.loop)
                devices = future.result(timeout=15)
                device_list = [f"{(d.name or 'Desconhecido')} ({d.address})" for d in devices]
                self.root.after(0, lambda: self.update_device_list(device_list))
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Erro no scan: {e}"))

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
        if not self.controller.is_connected:
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
                    # Busca por Neuro ou termos relacionados
                    if any(term in name_upper for term in ["NEURO", "NEUROBEEP"]):
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

        device_info = self.devices_combo.get()
        if "(" in device_info and ")" in device_info:
            address = device_info.split("(")[1].split(")")[0]
        else:
            address = "00:00:00:00:00:00"

        self.log_message(f"🔗 Conectando ao ESP32 BLE: {address}...")

        def connect_thread():
            try:
                future = asyncio.run_coroutine_threadsafe(self.controller.connect_by_address(address), self.controller.loop)
                success = future.result(timeout=20)
            except Exception as e:
                success = False
                self.root.after(0, lambda: self.log_message(f"❌ Erro na conexão: {e}"))
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
        def disc_thread():
            try:
                future = asyncio.run_coroutine_threadsafe(self.controller.disconnect(), self.controller.loop)
                future.result(timeout=10)
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"❌ Erro ao desconectar: {e}"))
            finally:
                self.root.after(0, lambda: self._after_disconnect_ui())

        threading.Thread(target=disc_thread, daemon=True).start()

    def _after_disconnect_ui(self):
        self.connection_status.config(text="❌ Desconectado", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        if self.telemetry_running:
            self.toggle_telemetry()
        self.log_message("🔌 Desconectado do ESP32")

    def send_command(self, command):
        """Envia comando para o ESP32"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return

        #self.log_message(f"📤 Enviando comando: {command}")
        try:
            self.controller.send_command(command)
        except Exception as e:
            self.log_message(f"❌ Erro ao enfileirar comando {command}: {e}")

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
            if not self.controller.is_connected:
                messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
                return
            else:
                self.telemetry_running = True
                self.telemetry_btn.config(text="📊 Parar Telemetria")
                self.log_message(f"📊 Telemetria iniciada ({self.telemetry_hz.get()} Hz)")
                threading.Thread(target=self.telemetry_loop, daemon=True).start()
        else:
            self.telemetry_running = False
            self.telemetry_btn.config(text="📊 Iniciar Telemetria")
            self.log_message("📊 Telemetria parada")
            
    def telemetry_loop(self):
        """Loop de telemetria - envia RQS: periodicamente"""
        while self.telemetry_running and self.controller.is_connected:
            # Solicita telemetria do ESP32
            self.controller.request_telemetry()
            
            # Aguarda baseado na taxa de Hz
            hz = self.telemetry_hz.get()
            interval = 1.0 / hz  # Converte Hz para intervalo em segundos
            time.sleep(interval)
    
    def on_telemetry_received(self, data):
        """Callback chamado quando telemetria é recebida do ESP32"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        formatted_data = f"[{timestamp}] {data}\n"
        
        # Parse dos dados para display
        telemetry_dict = TelemetryParser.parse(data)
        if telemetry_dict and hasattr(self, 'telemetry_display'):
            self.telemetry_display.update_data(telemetry_dict)
        
        # Atualiza GUI de forma thread-safe
        self.root.after(0, lambda: self.update_telemetry(formatted_data))
    
    def clear_telemetry_display(self):
        """Limpa o display de telemetria"""
        if hasattr(self, 'telemetry_display'):
            self.telemetry_display.clear_data()
            self.log_message("📊 Display de telemetria limpo")
            
    def update_telemetry(self, data):
        """Atualiza display de telemetria"""
        self.telemetry_text.insert(tk.END, data)
        self.telemetry_text.see(tk.END)
        
        # Limita o tamanho do texto
        lines = self.telemetry_text.get("1.0", tk.END).split("\n")
        if len(lines) > 100:  # Aumentado para 100 linhas
            self.telemetry_text.delete("1.0", "50.0")
            
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
    print("=" * 60)
    print("🤖 ESP32 BLE Controller - Neuro Robot")
    print("=" * 60)
    print("📊 Display de Telemetria em Tempo Real")
    print("=" * 60)
    
    root = tk.Tk()
    app = RobotGUI(root)

    def on_closing():
        if app.auto_scanning:
            app.stop_auto_scan()
        if app.telemetry_running:
            app.telemetry_running = False
        if app.controller.is_connected:
            app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
