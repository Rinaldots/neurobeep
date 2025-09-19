import serial
import csv
import time
import re
from datetime import datetime
import os

class MotorDataCollector:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, csv_filename=None):
        """
        Inicializa o coletor de dados do motor
        
        Args:
            port: Porta serial (ex: '/dev/ttyACM0' no Linux, 'COM3' no Windows)
            baudrate: Taxa de transmissão (115200 por padrão)
            csv_filename: Nome do arquivo CSV (se None, gera automaticamente)
        """
        self.port = port
        self.baudrate = baudrate
        
        # Gerar nome do arquivo automaticamente se não fornecido
        if csv_filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_filename = f"dados_motor_right_{timestamp}.csv"
        else:
            self.csv_filename = csv_filename
            
        self.serial_conn = None
        self.csv_file = None
        self.csv_writer = None
        
        # Novo regex para formato: PWM Left: 0Left Velocity (m/s): 0.00 PWM Right: 210Right Velocity (m/s): 0.00
        self.right_pattern = re.compile(r'PWM Right:\s*(\d+)Right Velocity \(m/s\):\s*([-\d\.]+)')
        
    def connect_serial(self):
        """Conecta à porta serial"""
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Conectado à porta {self.port} com baudrate {self.baudrate}")
            time.sleep(2)  # Aguardar estabilização da conexão
            return True
        except serial.SerialException as e:
            print(f"Erro ao conectar à porta serial: {e}")
            return False
    
    def setup_csv(self):
        """Configura o arquivo CSV"""
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp','datetime','pwm_right','vel_right'])
            print(f"Arquivo CSV criado: {self.csv_filename}")
            return True
        except Exception as e:
            print(f"Erro ao criar arquivo CSV: {e}")
            return False
    
    def parse_right(self, line: str):
        """
        Extrai dados do PWM Right e velocidade Right de uma linha
        
        Args:
            line: Linha de texto da serial
            
        Returns:
            tuple: (pwm_right, vel_right) ou None se não encontrar dados
        """
        match = self.right_pattern.search(line)
        if match:
            return int(match.group(1)), float(match.group(2))
        return None
    
    def collect_data(self, duration=None):
        """
        Coleta dados do motor Right (PWM e velocidade)
        
        Args:
            duration: Duração em segundos (None = infinito)
        """
        if not self.connect_serial():
            return
            
        if not self.setup_csv():
            return
            
        print("Iniciando coleta de dados (Right motor)...")
        start = time.time()
        count = 0
        
        try:
            while True:
                # Verificar se deve parar por tempo
                if duration and (time.time() - start) > duration:
                    break
                
                if self.serial_conn.in_waiting:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        parsed = self.parse_right(line)
                        if parsed:
                            pwm_r, vel_r = parsed
                            now = time.time()
                            dt_str = datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                            self.csv_writer.writerow([now, dt_str, pwm_r, vel_r])
                            self.csv_file.flush()
                            count += 1
                            if count % 10 == 0:
                                print(f"{count} | PWM_R={pwm_r} Vel_R={vel_r:.3f}")
                
                time.sleep(0.01)  # Pequeno delay para não sobrecarregar CPU
                
        except KeyboardInterrupt:
            print("\nInterrompido")
        
        finally:
            self.cleanup()
            print(f"Total linhas: {count}")
            print(f"Arquivo salvo: {self.csv_filename}")
    
    def cleanup(self):
        """Limpa recursos (fecha conexões e arquivos)"""
        if self.serial_conn:
            self.serial_conn.close()
            print("Conexão serial fechada")
            
        if self.csv_file:
            self.csv_file.close()
            print("Arquivo CSV fechado")

def main():
    # Configurações
    PORT = '/dev/ttyACM0'  # Ajustar conforme necessário
    BAUDRATE = 115200
    
    # Criar coletor
    collector = MotorDataCollector(port=PORT, baudrate=BAUDRATE)
    
    print("=== Coletor de Dados do Motor ===")
    print(f"Porta: {PORT}")
    print(f"Baudrate: {BAUDRATE}")
    print()
    
    # Iniciar coleta (infinita até Ctrl+C)
    collector.collect_data(duration=100)

if __name__ == "__main__":
    main()
