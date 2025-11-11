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
import struct
import numpy as np

SERVICE_UUID = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
CHAR_UUID    = "d5593e6b-3328-493a-b3c9-9814683d8e40"

class TelemetryParser:
    """Parser para dados de telemetria do ESP32 (formato binário ou texto)"""
    
    @staticmethod
    def parse(data):
        """Parse telemetry data - supports both binary and text format"""
        result = {}
        
        # Detect binary format by checking for 0xBEEF header
        if isinstance(data, (bytes, bytearray)) and len(data) >= 2:
            if data[0] == 0xBE and data[1] == 0xEF:
                return TelemetryParser.parse_binary(data)
        
        # Fall back to text format
        if isinstance(data, bytes):
            data_str = data.decode('utf-8', errors='ignore')
        else:
            data_str = str(data)
            
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
    
    @staticmethod
    def parse_binary(data):
        """
        Parse binary telemetry format from ESP32
        Format: [header:2][encoders:8][velocities:16][pwm:8][gains:8][odom_pos:12][odom_vel:12][imu:12][line:2][line_markers:6][gps:17][rfid:12]
        Total: ~125 bytes
        """
        result = {}
        
        try:
            offset = 2  # Skip 0xBEEF header
            
            # Encoders (2 x int32)
            enc_l, enc_r = struct.unpack_from('<ii', data, offset)
            result['ENC_L'] = enc_l
            result['ENC_R'] = enc_r
            offset += 8
            
            # Velocities (4 x float32)
            vel_l, vel_r, tgt_l, tgt_r = struct.unpack_from('<ffff', data, offset)
            result['VEL_L'] = vel_l
            result['VEL_R'] = vel_r
            RobotGUI.left_velocity = vel_l
            RobotGUI.right_velocity = vel_r
            result['TGT_L'] = tgt_l
            result['TGT_R'] = tgt_r
            offset += 16
            
            # PWM (2 x float32) - offset should be 26 here
            pwm_l, pwm_r = struct.unpack_from('<ff', data, offset)
            result['PWM_L'] = pwm_l
            result['PWM_R'] = pwm_r
            offset += 8
            
            # Gains (2 x float32) - offset should be 34 here
            gain_l, gain_r = struct.unpack_from('<ff', data, offset)
            result['GAIN_L'] = gain_l
            result['GAIN_R'] = gain_r
            offset += 8
            
            # Odometry position (3 x float32: x, y, theta) - offset should be 42 here
            odom_x, odom_y, odom_th = struct.unpack_from('<fff', data, offset)
            result['ODOM_X'] = odom_x
            result['ODOM_Y'] = odom_y
            result['ODOM_TH'] = odom_th
            offset += 12
            
            # Odometry velocity (3 x float32: vx, vy, omega)
            odom_vx, odom_vy, odom_w = struct.unpack_from('<fff', data, offset)
            result['ODOM_VX'] = odom_vx
            result['ODOM_VY'] = odom_vy
            result['ODOM_W'] = odom_w
            offset += 12
            
            # IMU accel (3 x float32)
            imu_ax, imu_ay, imu_az = struct.unpack_from('<fff', data, offset)
            result['IMU_AX'] = imu_ax
            result['IMU_AY'] = imu_ay
            result['IMU_AZ'] = imu_az
            offset += 12
            
            # Line sensor (1 x int16)
            line_dist, = struct.unpack_from('<h', data, offset)
            result['LINE_DIST'] = line_dist
            offset += 2
            
            # Line markers (1 x uint16 count + 1 x float32 distance)
            marker_count, = struct.unpack_from('<H', data, offset)
            result['MARKER_CNT'] = marker_count
            offset += 2
            
            marker_dist, = struct.unpack_from('<f', data, offset)
            result['MARKER_DIST'] = marker_dist
            offset += 4
            
            # GPS (4 x float32 + 1 x uint8)
            gps_lat, gps_lon, gps_alt, gps_spd = struct.unpack_from('<ffff', data, offset)
            result['GPS_LAT'] = gps_lat
            result['GPS_LNG'] = gps_lon  # Interface usa GPS_LNG, não GPS_LON
            result['GPS_ALT'] = gps_alt
            result['GPS_SPD'] = gps_spd
            offset += 16
            
            gps_valid, = struct.unpack_from('<B', data, offset)
            result['GPS_VAL'] = gps_valid  # Interface usa GPS_VAL, não GPS_VALID
            offset += 1
            
            # RFID (12-char string)
            rfid_bytes = data[offset:offset+12]
            rfid_str = rfid_bytes.decode('utf-8', errors='ignore').rstrip('\x00')
            if rfid_str:
                result['RFID'] = rfid_str
            offset += 12
            #print(data)
        except Exception as e:
            print(f"Erro ao decodificar telemetria binária: {e}")
        
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
        
        # Criar duas colunas
        self.column1 = ttk.Frame(self.scrollable_frame)
        self.column1.grid(row=0, column=0, sticky=(tk.N, tk.W, tk.E), padx=5)
        
        self.column2 = ttk.Frame(self.scrollable_frame)
        self.column2.grid(row=0, column=1, sticky=(tk.N, tk.W, tk.E), padx=5)
        
        # COLUNA 1 - Dados principais do robô
        self.create_group("🚗 Encoders", [
            ("ENC_L", "Encoder Esquerdo", 0, 1000),
            ("ENC_R", "Encoder Direito", 0, 1000)
        ], parent=self.column1)
        
        self.create_group("🏎️ Velocidades (m/s)", [
            ("VEL_L", "Velocidade Esquerda", -0.5, 0.5),
            ("VEL_R", "Velocidade Direita", -0.5, 0.5),
            ("TGT_L", "Target Esquerda", -0.5, 0.5),
            ("TGT_R", "Target Direita", -0.5, 0.5)
        ], parent=self.column1)
        
        self.create_group("⚡ PWM (0-255)", [
            ("PWM_L", "PWM Esquerdo", 0, 255),
            ("PWM_R", "PWM Direito", 0, 255),
            ("GAIN_L", "Gain Esquerdo", 0, 255),
            ("GAIN_R", "Gain Direito", 0, 255)
        ], parent=self.column1)
        
        self.create_group("📍 Odometria", [
            ("ODOM_X", "Posição X (m)", -5, 5),
            ("ODOM_Y", "Posição Y (m)", -5, 5),
            ("ODOM_TH", "Ângulo θ (rad)", -3.14, 3.14),
            ("ODOM_VX", "Velocidade X", -1, 1),
            ("ODOM_VY", "Velocidade Y", -1, 1),
            ("ODOM_W", "Velocidade Angular", -2, 2)
        ], parent=self.column1)
        
        # COLUNA 2 - Sensores
        self.create_group("📡 IMU Acelerômetro (m/s²)", [
            ("IMU_AX", "Aceleração X", -20, 20),
            ("IMU_AY", "Aceleração Y", -20, 20),
            ("IMU_AZ", "Aceleração Z", -20, 20)
        ], parent=self.column2)
        
        self.create_group("📏 Sensor de Linha", [
            ("LINE_DIST", "Distância", -3000, 3000)
        ], parent=self.column2)

        self.create_group("🟡 Marcadores de Linha", [
            ("MARKER_CNT", "Contador", 0, 100),
            ("MARKER_DIST", "Distância (m)", -3000, 3000)
        ], parent=self.column2)
        
        self.create_group("�🌍 GPS", [
            ("GPS_LAT", "Latitude", -90, 90),
            ("GPS_LNG", "Longitude", -180, 180),
            ("GPS_ALT", "Altitude (m)", 0, 1000),
            ("GPS_SPD", "Velocidade", 0, 100),
            ("GPS_VAL", "Válido", 0, 1)
        ], parent=self.column2)
        
    def create_group(self, title, fields, parent=None):
        """Cria um grupo de displays com título"""
        # Se parent não for especificado, usa scrollable_frame como padrão
        if parent is None:
            parent = self.scrollable_frame
            
        group_frame = ttk.LabelFrame(parent, text=title, padding=10)
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
        self.last_parsed_telemetry = {}
        self.telemetry_callback = None

        # Event loop dedicado em thread separada
        self.loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()

        # Placeholders inicializados no bootstrap assíncrono
        self.command_queue = None
        self.connected = None
        # Contador e lock para manter ordem entre comandos com mesma prioridade
        self._cmd_counter = 0
        self._cmd_lock = threading.Lock()
        # Flag para evitar enfiletrar múltiplos pedidos de telemetria simultâneos
        self._telemetry_queued = False

        # Inicializa estruturas assíncronas dentro do loop dedicado
        asyncio.run_coroutine_threadsafe(self._bootstrap_async(), self.loop)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def _bootstrap_async(self):
        # Criadas dentro do loop para evitar problemas de binding
        # Usa uma fila de prioridade: (priority, counter, message, response)
        self.command_queue = asyncio.PriorityQueue()
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
            # Agora suporta tanto formato binário quanto texto
            self.telemetry_data = data  # Mantém dados brutos (bytes ou string)
            self.last_parsed_telemetry = TelemetryParser.parse(data)
            
            # Chama callback se existir (para atualizar GUI)
            if self.telemetry_callback:
                self.telemetry_callback(self.last_parsed_telemetry)
        except Exception as e:
            print(f"Erro ao processar notificação: {e}")

    async def _command_worker(self):
        """Processa fila de comandos para evitar GATT congestionado"""
        while True:
            # Espera tuple (priority, count, message, response)
            try:
                priority, count, message, response = await self.command_queue.get()
            except Exception as e:
                print(f"⚠️ Erro ao obter comando da fila: {e}")
                await asyncio.sleep(0.05)
                continue

            try:
                await self._send_ble_async(message, response=response)
            except Exception as e:
                print(f"⚠️ Erro ao enviar '{message}': {e}")

            # Se for telemetria, libera a flag para permitir próximo pedido
            if isinstance(message, str) and message.startswith("RQS"):
                # Protege acesso concorrente
                try:
                    self._cmd_lock.acquire()
                    self._telemetry_queued = False
                finally:
                    self._cmd_lock.release()

            # Pequeno delay para não sobrecarregar GATT; mantido baixo para alta responsividade
            await asyncio.sleep(0.05)
            try:
                self.command_queue.task_done()
            except Exception:
                pass

    async def _send_ble_async(self, message, response=True):
        if not self.client or not self.client.is_connected:
            print("⚠️ Cliente BLE não está conectado, aguardando...")
            if self.connected:
                await self.connected.wait()

        data = message.encode("utf-8")
        # response flag permite enviar telemetria sem bloquear por confirmação quando apropriado
        await self.client.write_gatt_char(CHAR_UUID, data, response=response)

    def send_command(self, message):
        """Interface pública segura — pode ser chamada de qualquer thread"""
        # API mantida, mas agora enfileira com prioridade padrão (1)
        return self.send_command_with_priority(message, priority=1, response=True)

    def send_command_with_priority(self, message, priority=1, response=True):
        """Enfileira um comando com prioridade (menor valor = maior prioridade).
        Pode ser chamado de qualquer thread.
        """
        # Garante ordem entre itens com mesma prioridade
        with self._cmd_lock:
            count = self._cmd_counter
            self._cmd_counter += 1

            # Se for pedido de telemetria e já houver um pendente, ignora
            if isinstance(message, str) and message.startswith("RQS"):
                if self._telemetry_queued:
                    return False
                self._telemetry_queued = True

        # Coloca na fila de prioridade
        try:
            asyncio.run_coroutine_threadsafe(self.command_queue.put((priority, count, message, response)), self.loop)
            return True
        except Exception as e:
            print(f"❌ Erro ao enfileirar comando: {e}")
            return False

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
            # Telemetria tem prioridade baixa para não bloquear comandos de controle
            # usa response=False para reduzir espera por confirmação quando possível
            self.send_command_with_priority("RQS:", priority=10, response=False)
            return self.telemetry_data  # Retorna último dado recebido
        except Exception as e:
            print(f"Erro ao solicitar telemetria: {e}")
            return ""
    
    def get_current_speed(self):
        """Retorna a velocidade média atual do robô a partir da última telemetria."""
        vel_l = self.last_parsed_telemetry.get('VEL_L', 0)
        vel_r = self.last_parsed_telemetry.get('VEL_R', 0)
        return (vel_l + vel_r) / 2.0

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
        self.telemetry_hz = tk.IntVar(value=20)  # Taxa de telemetria em Hz
        self.pid_kp_var = tk.DoubleVar(value=0.50)
        self.pid_ki_var = tk.DoubleVar(value=50.0)
        self.pid_kd_var = tk.DoubleVar(value=25.0)

        # Janela e display de telemetria
        self.telemetry_window = None
        self.telemetry_display = None

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
        
        # Aba 2: Configurações
        settings_tab = ttk.Frame(notebook)
        notebook.add(settings_tab, text="⚙️ Configurações")

        # Frame principal da aba de configurações
        settings_frame = ttk.Frame(settings_tab, padding="10")
        settings_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        settings_tab.columnconfigure(0, weight=1)
        settings_tab.rowconfigure(0, weight=1)

        # Cria os frames de configuração
        self.create_settings_frame(settings_frame)

        # Configurar grid weights principal
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        
    def create_settings_frame(self, parent):
        """Cria a aba de configurações"""
        # Este frame pode ser dividido em colunas se necessário
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)

        # Frame de Calibração
        calibration_frame = ttk.LabelFrame(parent, text="🔬 Calibração", padding="10")
        calibration_frame.grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        calibration_frame.columnconfigure(0, weight=1)
        
        ttk.Button(calibration_frame, text="CALIBRATE_LINE_SENSORS", 
                  command=lambda: self.send_command("CMD:CALIBRATE_LINE_SENSORS")).grid(row=0, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(calibration_frame, text="CALIBRATE_IMU", 
                  command=lambda: self.send_command("CMD:CALIBRATE_IMU")).grid(row=1, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))

        # Frame de Ganhos PID
        pid_frame = ttk.LabelFrame(parent, text="⚙️ Ganhos PID", padding="10")
        pid_frame.grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        pid_frame.columnconfigure(0, weight=1)

        kp_pid_frame = ttk.Frame(pid_frame)
        kp_pid_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(kp_pid_frame, text="Kp:", width=4).pack(side=tk.LEFT)
        kp_pid_spin = ttk.Spinbox(kp_pid_frame, from_=0.0, to=10.0, increment=0.1, width=8,
                                  textvariable=self.pid_kp_var, format="%.2f")
        kp_pid_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ki_pid_frame = ttk.Frame(pid_frame)
        ki_pid_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(ki_pid_frame, text="Ki:", width=4).pack(side=tk.LEFT)
        ki_pid_spin = ttk.Spinbox(ki_pid_frame, from_=0.0, to=100.0, increment=0.1, width=8,
                                  textvariable=self.pid_ki_var, format="%.2f")
        ki_pid_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)

        kd_pid_frame = ttk.Frame(pid_frame)
        kd_pid_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(kd_pid_frame, text="Kd:", width=4).pack(side=tk.LEFT)
        kd_pid_spin = ttk.Spinbox(kd_pid_frame, from_=0.0, to=100.0, increment=0.1, width=8,
                                  textvariable=self.pid_kd_var, format="%.2f")
        kd_pid_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)

        ttk.Button(pid_frame, text="⚙️ Aplicar Ganhos PID",
                  command=self.apply_pid_gains).pack(fill=tk.X, padx=5, pady=5)

        ttk.Button(pid_frame, text="🤖 Auto-Tune PID",
                    command=self.start_pid_autotune).pack(fill=tk.X, padx=5, pady=2)

        # Frame do Seguidor de Linha
        line_follower_frame = ttk.LabelFrame(parent, text="🛤️ Seguidor de Linha", padding="10")
        line_follower_frame.grid(row=0, column=1, rowspan=2, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        line_follower_frame.columnconfigure(0, weight=1)

        speed_frame = ttk.Frame(line_follower_frame)
        speed_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(speed_frame, text="Vel:", width=4).pack(side=tk.LEFT)
        self.line_speed_var = tk.DoubleVar(value=0.3)
        speed_spin = ttk.Spinbox(speed_frame, from_=0.1, to=0.5, increment=0.05, width=6,
                                textvariable=self.line_speed_var, format="%.2f")
        speed_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        kp_frame = ttk.Frame(line_follower_frame)
        kp_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(kp_frame, text="Kp:", width=4).pack(side=tk.LEFT)
        self.line_kp_var = tk.DoubleVar(value=0.001)
        kp_spin = ttk.Spinbox(kp_frame, from_=0.0001, to=0.01, increment=0.0001, width=8,
                             textvariable=self.line_kp_var, format="%.4f")
        kp_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Button(line_follower_frame, text="⚙️ Aplicar Config. Seguidor", 
                  command=self.apply_line_config).pack(fill=tk.X, padx=5, pady=5)

        # Frame de Marcadores
        marker_frame = ttk.LabelFrame(parent, text="🎯 Marcadores", padding="10")
        marker_frame.grid(row=2, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        marker_frame.columnconfigure(0, weight=1)

        ttk.Button(marker_frame, text="🔄 Reset Marcadores", 
                  command=self.reset_markers).pack(fill=tk.X, padx=5, pady=2)
        
        spacing_frame = ttk.Frame(marker_frame)
        spacing_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(spacing_frame, text="Espaço (m):").pack(side=tk.LEFT)
        self.marker_spacing_var = tk.DoubleVar(value=0.5)
        spacing_spin = ttk.Spinbox(spacing_frame, from_=0.1, to=5.0, increment=0.1, width=6,
                                  textvariable=self.marker_spacing_var, format="%.1f")
        spacing_spin.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        ttk.Button(marker_frame, text="⚙️ Aplicar Espaçamento", 
                  command=self.apply_marker_spacing).pack(fill=tk.X, padx=5, pady=5)

        # Frame de Telemetria
        telemetry_cfg_frame = ttk.LabelFrame(parent, text="📊 Config. Telemetria", padding="10")
        telemetry_cfg_frame.grid(row=2, column=1, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        telemetry_cfg_frame.columnconfigure(0, weight=1)

        hz_frame = ttk.Frame(telemetry_cfg_frame)
        hz_frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(hz_frame, text="Taxa (Hz):").pack(side=tk.LEFT)
        hz_spinbox = ttk.Spinbox(hz_frame, from_=1, to=30, width=5, 
                                textvariable=self.telemetry_hz, state="readonly")
        hz_spinbox.pack(side=tk.LEFT, fill=tk.X, expand=True)

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
        
        self.line_follow_btn = ttk.Button(advanced_frame, text="▶️ Iniciar Seguidor", 
                  command=self.toggle_line_follower)
        self.line_follow_btn.grid(row=1, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))

        # Comandos de teste
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Button(advanced_frame, text="🧪 Teste Conexão", 
                  command=lambda: self.send_command("CMD:TEST")).grid(row=3, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        self.line_following_active = False
        
        # Telemetria
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.telemetry_btn = ttk.Button(advanced_frame, text="📊 Iniciar Telemetria", 
                                      command=self.toggle_telemetry)
        self.telemetry_btn.grid(row=5, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="📈 Abrir Janela de Telemetria",
                   command=self.open_telemetry_window).grid(row=6, column=0, padx=5, pady=2, sticky=(tk.W, tk.E))

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
    
    def on_telemetry_received(self, telemetry_dict):
        """Callback chamado quando telemetria é recebida do ESP32"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Log formatado dependendo do tipo de dados
        if telemetry_dict:
            # Formato binário
            formatted_data = f"[{timestamp}] 📦 Binary telemetry: {len(telemetry_dict)} fields\n"
        else:
            # Formato texto ou desconhecido
            formatted_data = f"[{timestamp}] {telemetry_dict}\n"
        
        if telemetry_dict and self.telemetry_display:
            self.telemetry_display.update_data(telemetry_dict)
        
        # Atualiza GUI de forma thread-safe
        self.root.after(0, lambda: self.update_telemetry(formatted_data))
    
    def open_telemetry_window(self):
        """Abre uma nova janela para o display de telemetria."""
        if self.telemetry_window and self.telemetry_window.winfo_exists():
            self.telemetry_window.lift()
            return

        self.telemetry_window = tk.Toplevel(self.root)
        self.telemetry_window.title("📊 Display de Telemetria")
        self.telemetry_window.geometry("800x600")
        self.telemetry_window.configure(bg='#2b2b2b')

        # Toolbar
        toolbar_frame = ttk.Frame(self.telemetry_window)
        toolbar_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(toolbar_frame, text="🧹 Limpar", 
                  command=self.clear_telemetry_display).pack(side=tk.LEFT, padx=5)

        # Frame para o display
        display_frame = ttk.Frame(self.telemetry_window)
        display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Cria o display
        self.telemetry_display = TelemetryDisplay(display_frame)
        self.log_message("📈 Janela de telemetria aberta.")

        # Limpa a referência quando a janela é fechada
        self.telemetry_window.protocol("WM_DELETE_WINDOW", self._on_telemetry_window_close)

    def _on_telemetry_window_close(self):
        """Callback para quando a janela de telemetria é fechada."""
        self.log_message("📈 Janela de telemetria fechada.")
        self.telemetry_display = None
        if self.telemetry_window:
            self.telemetry_window.destroy()
            self.telemetry_window = None

    def clear_telemetry_display(self):
        """Limpa o display de telemetria"""
        if self.telemetry_display:
            self.telemetry_display.clear_data()
            self.log_message("📊 Display de telemetria limpo")
        else:
            self.log_message("⚠️ Janela de telemetria não está aberta.")
    
    def toggle_line_follower(self):
        """Inicia/para o seguidor de linha"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        if not self.line_following_active:
            # Inicia seguidor de linha
            self.send_command("CMD:FOLLOW_LINE_START")
            self.line_following_active = True
            self.line_follow_btn.config(text="⏹️ Parar Seguidor")
            self.log_message("🛤️ Seguidor de linha iniciado")
        else:
            # Para seguidor de linha
            self.send_command("CMD:FOLLOW_LINE_STOP")
            self.line_following_active = False
            self.line_follow_btn.config(text="▶️ Iniciar Seguidor")
            self.log_message("🛤️ Seguidor de linha parado")
    
    def apply_line_config(self):
        """Aplica configuração do seguidor de linha"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        speed = self.line_speed_var.get()
        kp = self.line_kp_var.get()
        command = f"CMD:FOLLOW_LINE_CFG:{speed:.2f} {kp:.4f}"
        self.send_command(command)
        self.log_message(f"⚙️ Config seguidor: velocidade={speed:.2f}, Kp={kp:.4f}")
    
    def apply_pid_gains(self):
        """Aplica ganhos PID"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        kp = self.pid_kp_var.get()
        ki = self.pid_ki_var.get()
        kd = self.pid_kd_var.get()
        command = f"CMD:SET_KP_KI_KD:{kp:.2f} {ki:.2f} {kd:.2f}"
        self.send_command(command)
        self.log_message(f"⚙️ Ganhos PID aplicados: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f}")

    def start_pid_autotune(self):
        """Inicia o processo de auto-tune do PID em uma thread separada."""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return

        if not messagebox.askyesno(
            "Auto-Tune PID",
            "Isso iniciará o processo de auto-tune do PID. "
            "O robô pode se mover inesperadamente. Deseja continuar?"
        ):
            return

        # Inicia o processo em uma nova thread para não bloquear a GUI
        threading.Thread(target=self._pid_autotune_thread, daemon=True).start()


    def _pid_autotune_thread(self):
        """Lógica do auto-tune que roda em segundo plano."""
        self.log_message("🤖 Iniciando Auto-Tune do PID por Curva de Reação...")

        # --- Parâmetros do autotune ---
        test_speed = 0.15      # Velocidade do teste (saída de passo)
        duration = 30         # Aumentado para garantir que a curva 'S' se estabilize.
        interval = 1/20        # Intervalo de leitura do sensor em segundos

        response = []         # Salva os valores da velocidade medida
        t_values = []

        try:
            # Seta PID inicial baixo para evitar comportamento agressivo
            kp, ki, kd = 0.5, 50.0, 25.0
            self.send_command(f"CMD:SET_KP_KI_KD:{kp:.2f} {ki:.2f} {kd:.2f}")

            # Define velocidade constante (Passo de Teste)
            self.log_message(f"🚗 Definindo velocidade constante para autotune: {test_speed} m/s")
            self.send_command(f"CMD:VEL:{test_speed} {test_speed}")

            start_time = time.time()
            while time.time() - start_time < duration:
                current_speed = self.controller.get_current_speed()
                print(f"Velocidade atual: {current_speed:.2f} m/s")  # Debug
                response.append(current_speed)
                t_values.append(time.time() - start_time)
                time.sleep(interval)

            self.send_command("CMD:VEL:0 0")
            self.log_message("⏹️ Teste de velocidade finalizado.")

            response = np.array(response)
            t_values = np.array(t_values)

            steady_state_value = np.mean(response[int(len(response) * 0.8):])
            
            if steady_state_value == 0:
                self.log_message("⚠️ Auto-Tune falhou: Velocidade de estado estacionário zero.")
                return

            derivatives = np.diff(response) / np.diff(t_values)
            R = np.max(derivatives)

            if R <= 0:
                self.log_message("⚠️ Auto-Tune falhou: Não foi detectada subida de velocidade (R <= 0).")
                return

            max_slope_index = np.argmax(derivatives)
            
            # Ponto na curva S onde a inclinação é máxima
            point_max_slope = response[max_slope_index]
            time_max_slope = t_values[max_slope_index]
            
            L = time_max_slope - (point_max_slope / R)
            Kp_process = steady_state_value / test_speed

            kp = (1.2 / Kp_process) * (1 / L)
            ki = 2 * kp / (L * 1) # Tau_i = 2 * L
            kd = 0.5 * kp * L      # Tau_d = 0.5 * L
            ku_simplificado = 1.0 / (R * L) # Aproximação de um ganho crítico

            kp = 0.6 * ku_simplificado
            ki = 2 * kp / (4 * L) # Ki = Kp / Tau_i, onde Tau_i = 4L
            kd = kp * L / 1.1     # Kd = Kp * Tau_d, onde Tau_d = L/2 (usando ~1.1 para estabilidade)
            ku_processo = R * L / Kp_process # O que o seu código faz: amplitude/velocidade

            Kp_final = 0.6 / (R * L)
            Ti_final = 4 * L
            Td_final = L / 2
            
            kp = Kp_final
            ki = kp / Ti_final
            kd = kp * Td_final
            
            self.send_command(f"CMD:SET_KP_KI_KD:{kp:.2f} {ki:.2f} {kd:.2f}")
            self.log_message(f"✅ Auto-Tune finalizado (Curva S): KP={kp:.2f}, KI={ki:.2f}, KD={kd:.2f}")
            self.log_message(f"Parâmetros da Curva: Tempo Morto (L)={L:.2f}s, Inclinação (R)={R:.2f}")

            # Atualiza os valores na GUI
            self.root.after(0, lambda: self.pid_kp_var.set(round(kp, 2)))
            self.root.after(0, lambda: self.pid_ki_var.set(round(ki, 2)))
            self.root.after(0, lambda: self.pid_kd_var.set(round(kd, 2)))
            
        except ImportError:
            self.log_message("❌ Erro: a biblioteca 'numpy' é necessária para o auto-tune. Instale com 'pip install numpy'")
        except Exception as e:
            self.log_message(f"❌ Erro durante o cálculo do PID: {e}")

    def reset_markers(self):
        """Reseta contador de marcadores"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        self.send_command("CMD:MARKER_RESET")
        self.log_message("🎯 Marcadores resetados")
    
    def apply_marker_spacing(self):
        """Aplica espaçamento entre marcadores"""
        if not self.controller.is_connected:
            messagebox.showwarning("Aviso", "Conecte-se ao ESP32 primeiro")
            return
            
        spacing = self.marker_spacing_var.get()
        command = f"CMD:MARKER_SPACING:{spacing:.1f}"
        self.send_command(command)
        self.log_message(f"🎯 Espaçamento marcadores: {spacing:.1f} m")
            
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
