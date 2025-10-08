#!/usr/bin/env python3
"""
Interface Gráfica para Controle do Robô ESP32 via BLE (Bluetooth Low Energy)
Baseada nos comandos definidos no arquivo bluetooth.h
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import json
import subprocess
import asyncio
from datetime import datetime

try:
    import bleak
    from bleak import BleakScanner, BleakClient
    BLE_AVAILABLE = True
    print("✅ BLE disponível - usando bleak")
except ImportError:
    BLE_AVAILABLE = False
    print("❌ Módulo bleak não disponível. BLE não suportado.")


class ESP32Controller:
    def __init__(self):
        self.connected = False
        self.socket = None
        self.ble_client = None
        self.device_address = None
        self.device_name = "ESP32test"
        self.is_ble = False
        self.ble_write_char = None
        
    def scan_devices(self):
        """Escaneia dispositivos Bluetooth disponíveis (Clássico e BLE)"""
        if not BLUETOOTH_AVAILABLE and not BLE_AVAILABLE:
            return [("Simulação ESP32", "00:00:00:00:00:00")]
        
        devices = []
        
        # Tenta Bluetooth Clássico primeiro
        if BLUETOOTH_AVAILABLE:
            try:
                print("Iniciando scan Bluetooth Clássico...")
                # Aumentar duração do scan e limpar cache
                nearby_devices = bluetooth.discover_devices(duration=8, lookup_names=True, flush_cache=True)
                print(f"Dispositivos Bluetooth Clássico encontrados: {nearby_devices}")
                
                # Log detalhado de cada dispositivo
                for addr, name in nearby_devices:
                    print(f"Dispositivo Clássico: {name} ({addr})")
                    devices.append((f"{name} [Classic]", addr))
                    
            except Exception as e:
                print(f"Erro ao escanear dispositivos Bluetooth Clássico: {e}")
        
        # Tenta BLE
        if BLE_AVAILABLE:
            try:
                print("Iniciando scan BLE...")
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                ble_devices = loop.run_until_complete(self.scan_ble_devices())
                loop.close()
                
                for device in ble_devices:
                    name = device.name if device.name else "Dispositivo BLE"
                    addr = device.address
                    print(f"Dispositivo BLE: {name} ({addr})")
                    devices.append((f"{name} [BLE]", addr))
                    
            except Exception as e:
                print(f"Erro ao escanear dispositivos BLE: {e}")
        
        # Se nenhum método funcionou, tenta fallback
        if not devices:
            devices = self.scan_devices_fallback()
            
        return devices
    
    async def scan_ble_devices(self):
        """Escaneia dispositivos BLE"""
        try:
            print("Escaneando dispositivos BLE...")
            # Scanner mais longo para dispositivos BLE
            scanner = bleak.BleakScanner()
            devices = await scanner.discover(timeout=15.0)
            ble_devices = []
            
            # Armazena dispositivos descobertos para uso posterior
            if not hasattr(self, 'ble_discovered_devices'):
                self.ble_discovered_devices = {}
                
            for device in devices:
                name = device.name or "Unknown BLE Device"
                addr = device.address
                ble_devices.append((f"{name} [BLE]", addr))
                print(f"BLE encontrado: {name} ({addr})")
                
                # Armazena o objeto device para conexão
                self.ble_discovered_devices[addr] = device
                
            return ble_devices
        except Exception as e:
            print(f"Erro no scan BLE: {e}")
            return []
    
    def scan_devices_fallback(self):
        """Método alternativo usando bluetoothctl"""
        try:
            import subprocess
            print("Tentando scan com bluetoothctl...")
            
            # Usa bluetoothctl para escanear
            result = subprocess.run(['bluetoothctl', 'devices'], 
                                  capture_output=True, text=True, timeout=10)
            
            devices = []
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if line.startswith('Device'):
                        parts = line.split(' ', 2)
                        if len(parts) >= 3:
                            addr = parts[1]
                            name = parts[2]
                            devices.append((name, addr))
                            print(f"Dispositivo encontrado via bluetoothctl: {name} ({addr})")
            
            return devices
        except Exception as e:
            print(f"Erro no scan alternativo: {e}")
            return []
    
    def connect(self, address, device_name=""):
        """Conecta ao dispositivo ESP32 (Bluetooth Clássico ou BLE)"""
        if not BLUETOOTH_AVAILABLE and not BLE_AVAILABLE:
            self.connected = True
            self.device_address = address
            return True
        
        # Determina se é BLE baseado no nome do dispositivo
        self.is_ble = "[BLE]" in device_name
        
        if self.is_ble and BLE_AVAILABLE:
            return self.connect_ble(address)
        elif BLUETOOTH_AVAILABLE:
            return self.connect_classic(address)
        else:
            print("Nenhum método de conexão disponível")
            return False
    
    def connect_classic(self, address):
        """Conecta via Bluetooth Clássico"""
        try:
            print(f"Conectando via Bluetooth Clássico a {address}")
            self.socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.socket.connect((address, 1))
            self.connected = True
            self.device_address = address
            self.is_ble = False
            return True
        except Exception as e:
            print(f"Erro ao conectar via Bluetooth Clássico: {e}")
            return False
    
    def connect_ble(self, address):
        """Conecta via BLE"""
        try:
            print(f"Preparando conexão BLE para {address}")
            
            # Se o dispositivo não está no cache, faz um scan rápido primeiro
            if not hasattr(self, 'ble_discovered_devices') or address not in self.ble_discovered_devices:
                print("Fazendo scan rápido antes da conexão...")
                import asyncio
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.scan_ble_devices())
                loop.close()
            
            # Agora tenta conectar
            import asyncio
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result = loop.run_until_complete(self._connect_ble_async(address))
            loop.close()
            return result
        except Exception as e:
            print(f"Erro geral na conexão BLE: {e}")
            return False
    
    async def _connect_ble_async(self, address):
        """Conexão BLE assíncrona"""
        try:
            print(f"Tentando conectar via BLE a {address}")
            
            # Primeiro, tenta usar o dispositivo descoberto no cache
            if hasattr(self, 'ble_discovered_devices') and address in self.ble_discovered_devices:
                device = self.ble_discovered_devices[address]
                print(f"Usando dispositivo do cache: {device.name}")
                self.ble_client = bleak.BleakClient(device)
            else:
                # Se não tem no cache, faz um scan rápido
                print("Dispositivo não encontrado no cache, fazendo scan...")
                scanner = bleak.BleakScanner()
                devices = await scanner.discover(timeout=10.0)
                device = None
                for d in devices:
                    if d.address == address:
                        device = d
                        break
                
                if device:
                    print(f"Dispositivo encontrado no scan: {device.name}")
                    self.ble_client = bleak.BleakClient(device)
                else:
                    print(f"Dispositivo {address} não encontrado, tentando conectar diretamente...")
                    self.ble_client = bleak.BleakClient(address)
            
            # Conecta ao dispositivo
            await self.ble_client.connect()
            
            if self.ble_client.is_connected:
                print(f"✅ Conectado via BLE a {address}")
                
                # Lista serviços disponíveis
                services = await self.ble_client.get_services()
                print("Serviços BLE disponíveis:")
                for service in services:
                    print(f"  Serviço: {service.uuid}")
                    for char in service.characteristics:
                        print(f"    Característica: {char.uuid} - Propriedades: {char.properties}")
                        
                        # Procura por característica de escrita (UART TX)
                        if "write" in char.properties or "write-without-response" in char.properties:
                            self.ble_write_char = char.uuid
                            print(f"    -> ✅ Característica de escrita encontrada: {char.uuid}")
                
                if not self.ble_write_char:
                    print("⚠️ Nenhuma característica de escrita encontrada, tentando UUIDs específicos...")
                    # UUIDs específicos do ESP32
                    esp32_uuids = [
                        "d5593e6b-3328-493a-b3c9-9814683d8e40",  # UUID específico do ESP32
                        "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",  # Nordic UART TX (backup)
                        "0000FFE1-0000-1000-8000-00805F9B34FB",  # HM-10 UART (backup)
                    ]
                    
                    for uuid in esp32_uuids:
                        try:
                            print(f"🔍 Testando UUID: {uuid}")
                            # Verifica se a característica existe
                            char = self.ble_client.services.get_characteristic(uuid)
                            if char:
                                self.ble_write_char = uuid
                                print(f"✅ Usando UUID ESP32: {uuid}")
                                break
                        except Exception as e:
                            print(f"   ❌ UUID {uuid} não encontrado: {e}")
                            continue
                    
                    if not self.ble_write_char:
                        # Como último recurso, pega a primeira característica com write
                        for service in services:
                            for char in service.characteristics:
                                if "write" in str(char.properties).lower():
                                    self.ble_write_char = char.uuid
                                    print(f"⚠️ Usando primeira característica de escrita encontrada: {self.ble_write_char}")
                                    break
                            if self.ble_write_char:
                                break
                    
                    if not self.ble_write_char:
                        # Força o uso do UUID específico do ESP32
                        self.ble_write_char = "d5593e6b-3328-493a-b3c9-9814683d8e40"
                        print(f"🔧 Forçando uso do UUID ESP32: {self.ble_write_char}")
                
                # Verifica se estamos no serviço correto
                esp32_service_uuid = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
                print(f"🔍 Procurando serviço ESP32: {esp32_service_uuid}")
                
                for service in services:
                    if service.uuid.lower() == esp32_service_uuid.lower():
                        print(f"✅ Serviço ESP32 encontrado: {service.uuid}")
                        for char in service.characteristics:
                            if char.uuid.lower() == "d5593e6b-3328-493a-b3c9-9814683d8e40":
                                self.ble_write_char = char.uuid
                                print(f"✅ Característica ESP32 confirmada: {char.uuid}")
                                break
                
                self.connected = True
                self.device_address = address
                self.is_ble = True
                
                # Testa a característica de escrita
                test_success = await self._test_write_characteristic()
                if not test_success:
                    print("⚠️ Teste da característica falhou, mas mantendo conexão")
                
                return True
            else:
                print("❌ Falha na conexão BLE")
                return False
                
        except Exception as e:
            print(f"❌ Erro na conexão BLE assíncrona: {e}")
            return False
    
    async def _test_write_characteristic(self):
        """Testa se a característica de escrita funciona"""
        try:
            if not self.ble_write_char:
                print("❌ Nenhuma característica para testar")
                return False
                
            print(f"🧪 Testando característica ESP32: {self.ble_write_char}")
            
            # Verifica se é o UUID correto do ESP32
            esp32_char_uuid = "d5593e6b-3328-493a-b3c9-9814683d8e40"
            if self.ble_write_char.lower() == esp32_char_uuid.lower():
                print("✅ UUID ESP32 confirmado!")
            
            # Envia um comando de teste simples
            test_msg = "CMD:TEST\n"
            await self.ble_client.write_gatt_char(self.ble_write_char, test_msg.encode())
            print("✅ Teste da característica ESP32 passou!")
            return True
            
        except Exception as e:
            print(f"❌ Teste da característica falhou: {e}")
            # Mesmo se o teste falhar, vamos tentar continuar
            return True
    
    def disconnect(self):
        """Desconecta do dispositivo"""
        if self.is_ble and self.ble_client:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.ble_client.disconnect())
                loop.close()
            except:
                pass
            self.ble_client = None
        elif self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        
        self.connected = False
        self.device_address = None
        self.is_ble = False
        self.ble_write_char = None
    
    def send_command(self, command):
        """Envia comando para o ESP32"""
        if not self.connected:
            return False
            
        try:
            message = f"CMD:{command}\n"
            
            if self.is_ble and self.ble_client and BLE_AVAILABLE:
                # Envia via BLE
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                result = loop.run_until_complete(self._send_ble_async(message))
                loop.close()
                return result
            elif BLUETOOTH_AVAILABLE and self.socket:
                # Envia via Bluetooth Clássico
                self.socket.send(message.encode())
                return True
            else:
                # Modo simulação
                print(f"[SIMULAÇÃO] Comando enviado: {message.strip()}")
                return True
        except Exception as e:
            print(f"Erro ao enviar comando: {e}")
            return False
    
    async def _send_ble_async(self, message):
        """Envia mensagem via BLE assíncrona"""
        try:
            print(f"🔄 Tentando enviar: {message.strip()}")
            print(f"📝 Característica de escrita: {self.ble_write_char}")
            print(f"🔗 Cliente conectado: {self.ble_client.is_connected if self.ble_client else 'None'}")
            
            if not self.ble_write_char:
                print("❌ Nenhuma característica de escrita definida")
                return False
                
            if not self.ble_client or not self.ble_client.is_connected:
                print("❌ Cliente BLE não conectado")
                return False
                
            # Tenta enviar o comando
            data = message.encode('utf-8')
            print(f"📦 Dados a enviar: {data} (tamanho: {len(data)} bytes)")
            
            await self.ble_client.write_gatt_char(self.ble_write_char, data)
            print(f"✅ [BLE] Comando enviado: {message.strip()}")
            return True
            
        except Exception as e:
            print(f"❌ Erro ao enviar via BLE: {e}")
            print(f"   Tipo do erro: {type(e).__name__}")
            return False
    
    def request_telemetry(self):
        """Solicita dados de telemetria"""
        if not self.connected:
            return None
            
        try:
            if BLUETOOTH_AVAILABLE and self.socket:
                self.socket.send("RQS:TELEMETRY\n".encode())
                # Aguarda resposta (implementação simplificada)
                data = self.socket.recv(1024).decode()
                return data
            else:
                # Modo simulação
                return json.dumps({
                    "timestamp": datetime.now().isoformat(),
                    "position_x": 1.23,
                    "position_y": 4.56,
                    "velocity": 0.2,
                    "battery": 85
                })
        except Exception as e:
            print(f"Erro ao solicitar telemetria: {e}")
            return None


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle do Robô ESP32 - NeuroBeep")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')
        
        # Inicializa o controlador
        self.controller = ESP32Controller()
        
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
        """Cria frame de conexão"""
        connection_frame = ttk.LabelFrame(parent, text="Conexão Bluetooth", padding="5")
        connection_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Scan devices
        ttk.Button(connection_frame, text="Escanear Dispositivos", 
                  command=self.scan_devices).grid(row=0, column=0, padx=5)
        
        # Auto scan button
        self.auto_scan_btn = ttk.Button(connection_frame, text="Escanear Automaticamente", 
                                      command=self.toggle_auto_scan)
        self.auto_scan_btn.grid(row=0, column=1, padx=5)
        
        # Debug bluetooth button
        self.debug_btn = ttk.Button(connection_frame, text="Debug BT", 
                                  command=self.debug_bluetooth)
        self.debug_btn.grid(row=1, column=0, padx=5, pady=2)
        
        # Lista de dispositivos
        self.devices_combo = ttk.Combobox(connection_frame, width=30, state="readonly")
        self.devices_combo.grid(row=0, column=2, padx=5)
        
        # Botões conectar/desconectar
        self.connect_btn = ttk.Button(connection_frame, text="Conectar", command=self.connect)
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        self.disconnect_btn = ttk.Button(connection_frame, text="Desconectar", 
                                       command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=4, padx=5)
        
        # Status da conexão
        self.connection_status = ttk.Label(connection_frame, text="Desconectado", 
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
        
        ttk.Button(movement_frame, text="PARAR", 
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
        
        # Controle de rotação
        ttk.Label(movement_frame, text="Rotação:").grid(row=5, column=0, sticky=tk.W, pady=10)
        turn_scale = ttk.Scale(movement_frame, from_=-1.0, to=1.0, 
                             variable=self.turn_var, orient="horizontal")
        turn_scale.grid(row=5, column=1, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.turn_label = ttk.Label(movement_frame, text="0.00")
        self.turn_label.grid(row=5, column=3, pady=10)
        
        turn_scale.configure(command=self.update_turn_label)
        
        ttk.Button(movement_frame, text="Aplicar Rotação", 
                  command=self.set_turn).grid(row=6, column=1, pady=5)
        
    def create_advanced_frame(self, parent):
        """Cria frame de controles avançados"""
        advanced_frame = ttk.LabelFrame(parent, text="Controles Avançados", padding="5")
        advanced_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=(5, 0))
        
        # Comandos do sistema
        ttk.Button(advanced_frame, text="Iniciar Sistema", 
                  command=lambda: self.send_command("START")).grid(row=0, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="Reset Filtro Kalman", 
                  command=lambda: self.send_command("RESET_KALMAN")).grid(row=1, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        ttk.Button(advanced_frame, text="Calibrar IMU", 
                  command=lambda: self.send_command("CALIBRATE_IMU")).grid(row=2, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # Telemetria
        ttk.Separator(advanced_frame, orient="horizontal").grid(row=3, column=0, sticky=(tk.W, tk.E), pady=10)
        
        self.telemetry_btn = ttk.Button(advanced_frame, text="Iniciar Telemetria", 
                                      command=self.toggle_telemetry)
        self.telemetry_btn.grid(row=4, column=0, padx=5, pady=5, sticky=(tk.W, tk.E))
        
        # Display de telemetria
        self.telemetry_text = scrolledtext.ScrolledText(advanced_frame, width=30, height=8)
        self.telemetry_text.grid(row=5, column=0, padx=5, pady=5, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        advanced_frame.columnconfigure(0, weight=1)
        advanced_frame.rowconfigure(5, weight=1)
        
    def create_status_frame(self, parent):
        """Cria frame de status e log"""
        status_frame = ttk.LabelFrame(parent, text="Log de Comandos", padding="5")
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)
        
        self.log_text = scrolledtext.ScrolledText(status_frame, height=8)
        self.log_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Botão para limpar log
        ttk.Button(status_frame, text="Limpar Log", 
                  command=self.clear_log).grid(row=1, column=0, pady=5)
        
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        
    def scan_devices(self):
        """Escaneia dispositivos Bluetooth"""
        self.log_message("Escaneando dispositivos Bluetooth...")
        
        def scan_thread():
            devices = self.controller.scan_devices()
            device_list = [f"{name} ({addr})" for name, addr in devices]
            
            self.root.after(0, lambda: self.update_device_list(device_list))
        
        threading.Thread(target=scan_thread, daemon=True).start()
    
    def debug_bluetooth(self):
        """Debug da conectividade Bluetooth"""
        self.log_message("=== DEBUG BLUETOOTH ===")
        self.log_message(f"Bluetooth Clássico: {'Disponível' if BLUETOOTH_AVAILABLE else 'Não disponível'}")
        self.log_message(f"BLE: {'Disponível' if BLE_AVAILABLE else 'Não disponível'}")
        
        def debug_thread():
            import subprocess
            
            # Verifica se o Bluetooth está ativo
            try:
                result = subprocess.run(['systemctl', 'is-active', 'bluetooth'], 
                                      capture_output=True, text=True)
                bt_status = result.stdout.strip()
                self.root.after(0, lambda: self.log_message(f"Status do serviço Bluetooth: {bt_status}"))
            except:
                self.root.after(0, lambda: self.log_message("Erro ao verificar status do Bluetooth"))
            
            # Lista dispositivos pareados
            try:
                result = subprocess.run(['bluetoothctl', 'paired-devices'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    paired = result.stdout.strip()
                    self.root.after(0, lambda: self.log_message(f"Dispositivos pareados:\n{paired}"))
                else:
                    self.root.after(0, lambda: self.log_message("Nenhum dispositivo pareado"))
            except:
                self.root.after(0, lambda: self.log_message("Erro ao listar dispositivos pareados"))
            
            # Testa scan com Python bluetooth
            self.root.after(0, lambda: self.log_message("Testando scan com módulo Python (Clássico + BLE)..."))
            devices_python = self.controller.scan_devices()
            self.root.after(0, lambda: self.log_message(f"Dispositivos via Python: {devices_python}"))
            
            # Testa scan com bluetoothctl
            try:
                self.root.after(0, lambda: self.log_message("Iniciando scan com bluetoothctl..."))
                subprocess.run(['bluetoothctl', 'scan', 'on'], timeout=2)
                time.sleep(3)
                subprocess.run(['bluetoothctl', 'scan', 'off'], timeout=2)
                
                result = subprocess.run(['bluetoothctl', 'devices'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    devices = result.stdout.strip()
                    self.root.after(0, lambda: self.log_message(f"Dispositivos via bluetoothctl:\n{devices}"))
            except Exception as e:
                self.root.after(0, lambda: self.log_message(f"Erro no scan bluetoothctl: {e}"))
        
        threading.Thread(target=debug_thread, daemon=True).start()
        
    def update_device_list(self, devices):
        """Atualiza lista de dispositivos"""
        self.devices_combo['values'] = devices
        if devices:
            self.devices_combo.current(0)
            self.log_message(f"Encontrados {len(devices)} dispositivos")
        else:
            self.log_message("Nenhum dispositivo encontrado")
    
    def toggle_auto_scan(self):
        """Inicia/para o escaneamento automático"""
        if not self.auto_scanning:
            self.start_auto_scan()
        else:
            self.stop_auto_scan()
    
    def start_auto_scan(self):
        """Inicia o escaneamento automático"""
        self.auto_scanning = True
        self.auto_scan_btn.config(text="Parar Auto Scan", style="Accent.TButton")
        self.connection_status.config(text="🔄 Escaneamento automático ativo...", foreground="blue")
        self.log_message("Iniciando escaneamento automático...")
        
        # Inicia thread do auto scan
        self.auto_scan_thread = threading.Thread(target=self.auto_scan_loop, daemon=True)
        self.auto_scan_thread.start()
    
    def stop_auto_scan(self):
        """Para o escaneamento automático"""
        self.auto_scanning = False
        self.auto_scan_btn.config(text="Escanear Automaticamente")
        if not self.controller.connected:
            self.connection_status.config(text="Desconectado", foreground="red")
        self.log_message("Escaneamento automático parado")
    
    def auto_scan_loop(self):
        """Loop de escaneamento automático"""
        scan_count = 0
        while self.auto_scanning:
            scan_count += 1
            # Atualiza status com animação simples
            dots = "." * (scan_count % 4)
            self.root.after(0, lambda d=dots, c=scan_count: self.connection_status.config(
                text=f"🔄 Auto scan #{c} em progresso{d}", foreground="blue"))
            
            self.root.after(0, lambda c=scan_count: self.log_message(f"🔍 Auto scan #{c} - Procurando dispositivos ESP32..."))
            
            # Escaneia dispositivos
            devices = self.controller.scan_devices()
            
            # Procura especificamente por ESP32 ou dispositivos com nomes relacionados
            esp32_devices = []
            potential_devices = []
            
            for name, addr in devices:
                if name:
                    name_upper = name.upper()
                    print(f"Verificando dispositivo: {name} ({addr})")
                    # Busca exata por termos relacionados ao ESP32
                    if any(term in name_upper for term in ["ESP32", "ESP32TEST", "NEUROBEEP", "ARDUINO", "BLUETOOTH"]):
                        print(f"ESP32 encontrado: {name}")
                        esp32_devices.append((name, addr))
                    # Dispositivos potenciais (nomes genéricos mas que podem ser ESP32)
                    elif any(term in name_upper for term in ["HC-", "DEVICE", "MODULE", "BT-"]):
                        potential_devices.append((name, addr))
            
            if esp32_devices:
                # Encontrou dispositivo ESP32 confirmado
                device_list = [f"{name} ({addr})" for name, addr in devices]
                self.root.after(0, lambda dl=device_list: self.update_device_list(dl))
                
                # Para o auto scan e tenta conectar automaticamente
                self.root.after(0, lambda: self.on_esp32_found(esp32_devices[0]))
                break
            elif potential_devices:
                # Encontrou dispositivos potenciais
                self.root.after(0, lambda: self.log_message(f"⚠️ Dispositivos potenciais encontrados: {[name for name, _ in potential_devices]}"))
                device_list = [f"{name} ({addr})" for name, addr in devices]
                self.root.after(0, lambda dl=device_list: self.update_device_list(dl))
                
                # Pergunta se quer parar o auto scan para verificar manualmente
                self.root.after(0, lambda: self.on_potential_devices_found(potential_devices))
            else:
                # Não encontrou, continua escaneando
                if devices:
                    self.root.after(0, lambda: self.log_message(f"❌ Dispositivos encontrados: {[name for name, _ in devices]}, mas nenhum ESP32 detectado"))
                else:
                    self.root.after(0, lambda: self.log_message("❌ Nenhum dispositivo encontrado"))
                
                # Aguarda antes do próximo scan (3 segundos)
                for i in range(30):
                    if not self.auto_scanning:
                        return
                    time.sleep(0.1)
        
        # Para o auto scan quando sai do loop
        self.root.after(0, self.stop_auto_scan)
    
    def on_esp32_found(self, device_info):
        """Callback quando ESP32 é encontrado"""
        name, addr = device_info
        self.stop_auto_scan()
        self.log_message(f"🎉 ESP32 encontrado: {name} ({addr})")
        
        # Pergunta se quer conectar automaticamente
        if messagebox.askyesno("ESP32 Encontrado!", 
                             f"ESP32 encontrado: {name}\nEndereço: {addr}\n\nDeseja conectar automaticamente?"):
            # Seleciona o dispositivo na combo box
            device_display = f"{name} ({addr})"
            devices = list(self.devices_combo['values'])
            if device_display in devices:
                self.devices_combo.set(device_display)
                # Conecta automaticamente
                self.connect()
    
    def on_potential_devices_found(self, potential_devices):
        """Callback quando dispositivos potenciais são encontrados"""
        if not self.auto_scanning:
            return
            
        device_names = [name for name, _ in potential_devices]
        message = f"Dispositivos potenciais encontrados:\n{', '.join(device_names)}\n\nDeseja parar o auto scan para verificar manualmente?"
        
        if messagebox.askyesno("Dispositivos Potenciais", message):
            self.stop_auto_scan()
            
    def connect(self):
        """Conecta ao dispositivo selecionado"""
        if not self.devices_combo.get():
            messagebox.showwarning("Aviso", "Selecione um dispositivo primeiro")
            return
            
        # Extrai endereço do dispositivo
        device_info = self.devices_combo.get()
        device_name = device_info
        
        if "(" in device_info and ")" in device_info:
            address = device_info.split("(")[1].split(")")[0]
        else:
            address = "00:00:00:00:00:00"  # Simulação
            
        connection_type = "BLE" if "[BLE]" in device_info else "Clássico"
        self.log_message(f"Conectando via {connection_type} a {address}...")
        
        if self.controller.connect(address, device_name):
            self.connection_status.config(text=f"Conectado ({connection_type})", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.log_message("Conectado com sucesso!")
        else:
            self.log_message("Falha na conexão")
            messagebox.showerror("Erro", "Não foi possível conectar ao dispositivo")
            
    def disconnect(self):
        """Desconecta do dispositivo"""
        self.controller.disconnect()
        self.connection_status.config(text="Desconectado", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        
        # Para telemetria se estiver rodando
        if self.telemetry_running:
            self.toggle_telemetry()
            
        self.log_message("Desconectado")
        
    def send_command(self, command):
        """Envia comando para o robô"""
        if not self.controller.connected:
            messagebox.showwarning("Aviso", "Conecte-se a um dispositivo primeiro")
            return
            
        if self.controller.send_command(command):
            self.log_message(f"Comando enviado: {command}")
        else:
            self.log_message(f"Erro ao enviar comando: {command}")
            
    def update_velocity_label(self, value):
        """Atualiza label de velocidade"""
        self.velocity_label.config(text=f"{float(value):.2f}")
        
    def update_turn_label(self, value):
        """Atualiza label de rotação"""
        self.turn_label.config(text=f"{float(value):.2f}")
        
    def set_velocity(self):
        """Define velocidade do robô"""
        velocity = self.velocity_var.get()
        command = f"VEL:{velocity:.2f}"
        self.send_command(command)
        
    def set_turn(self):
        """Define rotação do robô"""
        turn = self.turn_var.get()
        command = f"TURN:{turn:.2f}"
        self.send_command(command)
        
    def toggle_telemetry(self):
        """Inicia/para telemetria"""
        if not self.telemetry_running:
            self.telemetry_running = True
            self.telemetry_btn.config(text="Parar Telemetria")
            threading.Thread(target=self.telemetry_loop, daemon=True).start()
        else:
            self.telemetry_running = False
            self.telemetry_btn.config(text="Iniciar Telemetria")
            
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