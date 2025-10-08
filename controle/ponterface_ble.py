#!/usr/bin/env python3
"""
Interface Gráfica para Controle do Robô ESP32 via BLE (Bluetooth Low Energy)
Versão focada apenas em BLE
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


class ESP32BLEController:
    def __init__(self):
        self.connected = False
        self.ble_client = None
        self.device_address = None
        self.device_name = "ESP32test"
        self.ble_write_char = None
        self.ble_discovered_devices = {}
        
    def scan_devices(self):
        """Escaneia dispositivos BLE disponíveis"""
        if not BLE_AVAILABLE:
            return [("Simulação ESP32 [BLE]", "00:00:00:00:00:00")]
        
        try:
            print("🔍 Iniciando scan BLE...")
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            ble_devices = loop.run_until_complete(self.scan_ble_devices())
            loop.close()
            print(f"✅ Scan BLE concluído: {len(ble_devices)} dispositivos encontrados")
            return ble_devices
        except Exception as e:
            print(f"❌ Erro ao escanear dispositivos BLE: {e}")
            return []
    
    async def scan_ble_devices(self):
        """Escaneia dispositivos BLE"""
        try:
            print("Escaneando dispositivos BLE...")
            # Scanner mais longo para dispositivos BLE
            scanner = BleakScanner()
            devices = await scanner.discover(timeout=15.0)
            ble_devices = []
            
            # Limpa cache anterior
            self.ble_discovered_devices = {}
                
            for device in devices:
                name = device.name or "Unknown BLE Device"
                addr = device.address
                ble_devices.append((f"{name} [BLE]", addr))
                print(f"📱 BLE encontrado: {name} ({addr})")
                
                # Armazena o objeto device para conexão
                self.ble_discovered_devices[addr] = device
                
            return ble_devices
        except Exception as e:
            print(f"❌ Erro no scan BLE: {e}")
            return []
    
    def connect(self, address):
        """Conecta ao dispositivo ESP32 BLE"""
        if not BLE_AVAILABLE:
            self.connected = True
            self.device_address = address
            return True
        
        return self.connect_ble(address)
    
    def connect_ble(self, address):
        """Conecta via BLE"""
        try:
            print(f"🔗 Preparando conexão BLE para {address}")
            
            # Se o dispositivo não está no cache, faz um scan rápido primeiro
            if address not in self.ble_discovered_devices:
                print("📡 Fazendo scan rápido antes da conexão...")
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.scan_ble_devices())
                loop.close()
            
            # Agora tenta conectar
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            result = loop.run_until_complete(self._connect_ble_async(address))
            loop.close()
            return result
        except Exception as e:
            print(f"❌ Erro geral na conexão BLE: {e}")
            return False
    
    async def _connect_ble_async(self, address):
        """Conexão BLE assíncrona"""
        try:
            print(f"🔄 Tentando conectar via BLE a {address}")
            
            # Primeiro, tenta usar o dispositivo descoberto no cache
            if address in self.ble_discovered_devices:
                device = self.ble_discovered_devices[address]
                print(f"📋 Usando dispositivo do cache: {device.name}")
                self.ble_client = BleakClient(device)
            else:
                # Se não tem no cache, faz um scan rápido
                print("🔍 Dispositivo não encontrado no cache, fazendo scan...")
                scanner = BleakScanner()
                devices = await scanner.discover(timeout=10.0)
                device = None
                for d in devices:
                    if d.address == address:
                        device = d
                        break
                
                if device:
                    print(f"✅ Dispositivo encontrado no scan: {device.name}")
                    self.ble_client = BleakClient(device)
                else:
                    print(f"⚠️ Dispositivo {address} não encontrado, tentando conectar diretamente...")
                    self.ble_client = BleakClient(address)
            
            # Conecta ao dispositivo
            await self.ble_client.connect()
            
            if self.ble_client.is_connected:
                print(f"✅ Conectado via BLE a {address}")
                
                # Lista serviços disponíveis e procura características de escrita
                await self._discover_characteristics()
                
                self.connected = True
                self.device_address = address
                return True
            else:
                print("❌ Falha na conexão BLE")
                return False
                
        except Exception as e:
            print(f"❌ Erro na conexão BLE assíncrona: {e}")
            return False
    
    async def _discover_characteristics(self):
        """Descobre características BLE para comunicação"""
        try:
            services = await self.ble_client.get_services()
            print("📋 Serviços BLE disponíveis:")
            
            for service in services:
                print(f"  🔧 Serviço: {service.uuid}")
                for char in service.characteristics:
                    print(f"    📝 Característica: {char.uuid} - Propriedades: {char.properties}")
                    
                    # Procura por característica de escrita (UART TX)
                    if "write" in char.properties or "write-without-response" in char.properties:
                        self.ble_write_char = char.uuid
                        print(f"    -> ✅ Característica de escrita encontrada: {char.uuid}")
            
            if not self.ble_write_char:
                print("⚠️ Nenhuma característica de escrita encontrada, tentando UUIDs padrão...")
                # Tenta usar características UART padrão
                uart_uuids = [
                    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",  # Nordic UART TX
                    "0000FFE1-0000-1000-8000-00805F9B34FB",  # HM-10 UART
                    "49535343-8841-43F4-A8D4-ECBE34729BB3",  # Outro UART comum
                ]
                
                for uuid in uart_uuids:
                    try:
                        # Verifica se a característica existe
                        char = self.ble_client.services.get_characteristic(uuid)
                        if char:
                            self.ble_write_char = uuid
                            print(f"✅ Usando UART padrão: {uuid}")
                            break
                    except:
                        continue
                
                if not self.ble_write_char:
                    # Usa o primeiro UUID como fallback
                    self.ble_write_char = uart_uuids[0]
                    print(f"⚠️ Usando UUID fallback: {self.ble_write_char}")
                    
        except Exception as e:
            print(f"❌ Erro ao descobrir características: {e}")
    
    def disconnect(self):
        """Desconecta do dispositivo"""
        if self.ble_client:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.ble_client.disconnect())
                loop.close()
                print("🔌 Desconectado do BLE")
            except Exception as e:
                print(f"Erro ao desconectar BLE: {e}")
            self.ble_client = None
        
        self.connected = False
        self.device_address = None
        self.ble_write_char = None
    
    def send_command(self, command):
        """Envia comando para o ESP32"""
        if not self.connected:
            return False
            
        try:
            message = f"CMD:{command}\\n"
            
            if BLE_AVAILABLE and self.ble_client:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                result = loop.run_until_complete(self._send_ble_async(message))
                loop.close()
                return result
            else:
                print(f"[SIMULAÇÃO] Comando enviado: {message}")
                return True
        except Exception as e:
            print(f"❌ Erro ao enviar comando: {e}")
            return False
    
    async def _send_ble_async(self, message):
        """Envia mensagem via BLE assíncrona"""
        try:
            if self.ble_write_char and self.ble_client.is_connected:
                data = message.encode('utf-8')
                await self.ble_client.write_gatt_char(self.ble_write_char, data)
                print(f"📤 Comando BLE enviado: {message.strip()}")
                return True
            else:
                print("❌ Sem característica de escrita ou não conectado")
                return False
        except Exception as e:
            print(f"❌ Erro ao enviar via BLE: {e}")
            return False
    
    def request_telemetry(self):
        """Solicita dados de telemetria"""
        if not self.connected:
            return None
            
        try:
            # Modo simulação para telemetria
            return json.dumps({
                "timestamp": datetime.now().isoformat(),
                "position_x": 1.23,
                "position_y": 4.56,
                "velocity": 0.2,
                "battery": 85
            })
        except Exception as e:
            print(f"❌ Erro ao solicitar telemetria: {e}")
            return None


class RobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle do Robô ESP32 BLE - NeuroBeep")
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
        connection_frame = ttk.LabelFrame(parent, text="Conexão BLE", padding="5")
        connection_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)
        
        # Scan devices
        ttk.Button(connection_frame, text="Escanear BLE", 
                  command=self.scan_devices).grid(row=0, column=0, padx=5)
        
        # Auto scan button
        self.auto_scan_btn = ttk.Button(connection_frame, text="Auto Scan BLE", 
                                      command=self.toggle_auto_scan)
        self.auto_scan_btn.grid(row=0, column=1, padx=5)
        
        # Debug button
        ttk.Button(connection_frame, text="Debug BLE", 
                  command=self.debug_bluetooth).grid(row=0, column=2, padx=5)
        
        # Lista de dispositivos
        self.devices_combo = ttk.Combobox(connection_frame, width=30, state="readonly")
        self.devices_combo.grid(row=0, column=3, padx=5)
        
        # Botões conectar/desconectar
        self.connect_btn = ttk.Button(connection_frame, text="Conectar BLE", command=self.connect)
        self.connect_btn.grid(row=0, column=4, padx=5)
        
        self.disconnect_btn = ttk.Button(connection_frame, text="Desconectar", 
                                       command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=5, padx=5)
        
        # Status da conexão
        self.connection_status = ttk.Label(connection_frame, text="Desconectado BLE", 
                                         foreground="red")
        self.connection_status.grid(row=1, column=0, columnspan=6, pady=5)
        
    def create_movement_frame(self, parent):
        """Cria frame de controles de movimento"""
        movement_frame = ttk.LabelFrame(parent, text="Controles de Movimento", padding="5")
        movement_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5, padx=(0, 5))
        
        # Botões de direção em layout de cruz
        ttk.Button(movement_frame, text="↑\\nFRENTE", 
                  command=lambda: self.send_command("FRENTE")).grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="←\\nESQUERDA", 
                  command=lambda: self.send_command("ESQUERDA")).grid(row=1, column=0, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="PARAR", 
                  command=lambda: self.send_command("PARAR")).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="→\\nDIREITA", 
                  command=lambda: self.send_command("DIREITA")).grid(row=1, column=2, padx=5, pady=5)
        
        ttk.Button(movement_frame, text="↓\\nTRÁS", 
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
        """Escaneia dispositivos BLE"""
        self.log_message("🔍 Escaneando dispositivos BLE...")
        
        def scan_thread():
            devices = self.controller.scan_devices()
            device_list = [f"{name} ({addr})" for name, addr in devices]
            self.root.after(0, lambda: self.update_device_list(device_list))
        
        threading.Thread(target=scan_thread, daemon=True).start()
        
    def debug_bluetooth(self):
        """Debug BLE específico"""
        self.log_message("=== DEBUG BLE ===")
        
        def debug_thread():
            debug_info = []
            debug_info.append("=== DEBUG BLE ===")
            debug_info.append(f"BLE disponível: {BLE_AVAILABLE}")
            debug_info.append("")
            
            # Testa BLE
            if BLE_AVAILABLE:
                try:
                    debug_info.append("--- BLE (Bluetooth Low Energy) ---")
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                    
                    # Scan mais longo para BLE
                    async def scan_ble_debug():
                        scanner = BleakScanner()
                        devices = await scanner.discover(timeout=10.0)
                        return devices
                    
                    ble_devices = loop.run_until_complete(scan_ble_debug())
                    loop.close()
                    
                    debug_info.append(f"Dispositivos BLE encontrados: {len(ble_devices)}")
                    for device in ble_devices:
                        name = device.name or "Nome desconhecido"
                        addr = device.address
                        debug_info.append(f"  {name} ({addr})")
                        if "ESP32" in name.upper():
                            debug_info.append(f"    -> ✅ ESP32 BLE DETECTADO!")
                            
                except Exception as e:
                    debug_info.append(f"❌ Erro no BLE: {e}")
            
            # Mostra resultado na GUI
            debug_text = "\\n".join(debug_info)
            self.root.after(0, lambda: self.show_debug_result(debug_text))
        
        threading.Thread(target=debug_thread, daemon=True).start()
    
    def show_debug_result(self, debug_text):
        """Mostra resultado do debug"""
        debug_window = tk.Toplevel(self.root)
        debug_window.title("Debug BLE")
        debug_window.geometry("600x400")
        
        text_widget = scrolledtext.ScrolledText(debug_window, wrap=tk.WORD)
        text_widget.pack(expand=True, fill='both', padx=10, pady=10)
        text_widget.insert(tk.END, debug_text)
        text_widget.config(state=tk.DISABLED)
        
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
        self.connection_status.config(text="🔄 Auto scan BLE ativo...", foreground="blue")
        self.log_message("🚀 Iniciando auto scan BLE...")
        
        # Inicia thread do auto scan
        self.auto_scan_thread = threading.Thread(target=self.auto_scan_loop, daemon=True)
        self.auto_scan_thread.start()
    
    def stop_auto_scan(self):
        """Para o escaneamento automático"""
        self.auto_scanning = False
        self.auto_scan_btn.config(text="Auto Scan BLE")
        if not self.controller.connected:
            self.connection_status.config(text="Desconectado BLE", foreground="red")
        self.log_message("⏹️ Auto scan BLE parado")
    
    def auto_scan_loop(self):
        """Loop de escaneamento automático BLE"""
        scan_count = 0
        while self.auto_scanning:
            scan_count += 1
            # Atualiza status com animação simples
            dots = "." * (scan_count % 4)
            self.root.after(0, lambda d=dots, c=scan_count: self.connection_status.config(
                text=f"🔄 Auto scan BLE #{c}{d}", foreground="blue"))
            
            self.root.after(0, lambda c=scan_count: self.log_message(f"🔍 Auto scan BLE #{c} - Procurando ESP32..."))
            
            # Escaneia dispositivos
            devices = self.controller.scan_devices()
            
            # Procura especificamente por ESP32
            esp32_devices = []
            
            for name, addr in devices:
                if name:
                    name_upper = name.upper()
                    # Busca por ESP32 ou termos relacionados
                    if any(term in name_upper for term in ["ESP32", "NEUROBEEP", "ESP32TEST"]):
                        esp32_devices.append((name, addr))
            
            if esp32_devices:
                # Encontrou dispositivo ESP32 BLE
                device_list = [f"{name} ({addr})" for name, addr in devices]
                self.root.after(0, lambda dl=device_list: self.update_device_list(dl))
                
                # Para o auto scan e tenta conectar automaticamente
                self.root.after(0, lambda: self.on_esp32_found(esp32_devices[0]))
                break
            else:
                # Não encontrou, continua escaneando
                if devices:
                    self.root.after(0, lambda: self.log_message(f"📱 Dispositivos BLE encontrados: {[name for name, _ in devices]}, mas nenhum ESP32"))
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
                             f"ESP32 BLE encontrado: {name}\\nEndereço: {addr}\\n\\nDeseja conectar automaticamente?"):
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
            messagebox.showwarning("Aviso", "Selecione um dispositivo BLE primeiro")
            return
            
        # Extrai endereço do dispositivo
        device_info = self.devices_combo.get()
        if "(" in device_info and ")" in device_info:
            address = device_info.split("(")[1].split(")")[0]
        else:
            address = "00:00:00:00:00:00"  # Simulação
            
        self.log_message(f"🔗 Conectando via BLE a {address}...")
        
        def connect_thread():
            success = self.controller.connect(address)
            self.root.after(0, lambda: self.on_connection_result(success))
        
        threading.Thread(target=connect_thread, daemon=True).start()
    
    def on_connection_result(self, success):
        """Callback do resultado da conexão"""
        if success:
            self.connection_status.config(text="✅ Conectado BLE", foreground="green")
            self.connect_btn.config(state="disabled")
            self.disconnect_btn.config(state="normal")
            self.log_message("🎉 Conectado com sucesso via BLE!")
        else:
            self.log_message("❌ Falha na conexão BLE")
            messagebox.showerror("Erro", "Não foi possível conectar ao dispositivo BLE")
            
    def disconnect(self):
        """Desconecta do dispositivo"""
        self.controller.disconnect()
        self.connection_status.config(text="Desconectado BLE", foreground="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        
        # Para telemetria se estiver rodando
        if self.telemetry_running:
            self.toggle_telemetry()
            
        self.log_message("🔌 Desconectado do BLE")
        
    def send_command(self, command):
        """Envia comando para o robô"""
        if not self.controller.connected:
            messagebox.showwarning("Aviso", "Conecte-se a um dispositivo BLE primeiro")
            return
            
        def send_thread():
            success = self.controller.send_command(command)
            if success:
                self.root.after(0, lambda: self.log_message(f"📤 Comando enviado: {command}"))
            else:
                self.root.after(0, lambda: self.log_message(f"❌ Erro ao enviar comando: {command}"))
        
        threading.Thread(target=send_thread, daemon=True).start()
            
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
                self.root.after(0, lambda: self.update_telemetry(f"[{timestamp}] {data}\\n"))
            time.sleep(1)
            
    def update_telemetry(self, data):
        """Atualiza display de telemetria"""
        self.telemetry_text.insert(tk.END, data)
        self.telemetry_text.see(tk.END)
        
        # Limita o tamanho do texto
        lines = self.telemetry_text.get("1.0", tk.END).split("\\n")
        if len(lines) > 50:
            self.telemetry_text.delete("1.0", "10.0")
            
    def log_message(self, message):
        """Adiciona mensagem ao log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
        
        # Limita o tamanho do log
        lines = self.log_text.get("1.0", tk.END).split("\\n")
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
