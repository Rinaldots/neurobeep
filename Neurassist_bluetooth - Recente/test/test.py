import bluetooth
import socket
import time

# Função para debug e verificação do sistema
def check_bluetooth_system():
    """Verifica se o sistema Bluetooth está funcionando"""
    try:
        # Testa se o adaptador Bluetooth está disponível
        test_devices = bluetooth.discover_devices(duration=1, lookup_names=False)
        print("✓ Adaptador Bluetooth está funcionando")
        return True
    except Exception as e:
        print(f"✗ Erro no adaptador Bluetooth: {e}")
        print("Tente executar como administrador: sudo python3 test.py")
        return False

# Verificação inicial
print("=== Diagnóstico Bluetooth ===")
if not check_bluetooth_system():
    exit(1)

target_name = "esp32"
target_address = None

print(f"\nProcurando dispositivo: {target_name}")
print("Escaneando dispositivos Bluetooth (20 segundos)...")

# Primeira tentativa: descoberta com nomes
try:
    nearby_devices = bluetooth.discover_devices(duration=20, lookup_names=True)
    print("Dispositivos encontrados com nomes:", nearby_devices)
except Exception as e:
    print(f"Erro na descoberta com nomes: {e}")
    # Segunda tentativa: descoberta sem nomes, depois busca nomes
    try:
        print("Tentando descoberta sem nomes...")
        device_addresses = bluetooth.discover_devices(duration=20, lookup_names=False)
        print(f"Endereços encontrados: {device_addresses}")
        
        nearby_devices = []
        for addr in device_addresses:
            try:
                name = bluetooth.lookup_name(addr, timeout=5)
                if name:
                    nearby_devices.append((addr, name))
                    print(f"Dispositivo: {name} ({addr})")
                else:
                    nearby_devices.append((addr, "Unknown"))
                    print(f"Dispositivo sem nome: {addr}")
            except Exception as e:
                print(f"Erro ao buscar nome para {addr}: {e}")
                nearby_devices.append((addr, "Error"))
    except Exception as e2:
        print(f"Erro na segunda tentativa: {e2}")
        nearby_devices = []

# Procura o dispositivo ESP32
for btaddr, btname in nearby_devices:
    print(f"Verificando: {btname} ({btaddr})")
    if btname and target_name.lower() in btname.lower():
        target_address = btaddr
        print(f"✓ Encontrado dispositivo compatível: {btname}")
        break

if target_address is None:
    # Última tentativa: procurar por qualquer dispositivo com "esp" no nome
    print(f"\nDispositivo exato '{target_name}' não encontrado.")
    print("Procurando por dispositivos com 'esp' no nome...")
    for btaddr, btname in nearby_devices:
        if btname and "esp" in btname.lower():
            print(f"Dispositivo ESP encontrado: {btname} ({btaddr})")
            response = input(f"Tentar conectar ao {btname}? (s/n): ")
            if response.lower() in ['s', 'sim', 'y', 'yes']:
                target_address = btaddr
                break

if target_address is not None:
    print(f"✓ Tentando conectar ao dispositivo: {target_address}")
    
    serverMACAddress = target_address
    port = 1

    try:
        print("Criando socket Bluetooth...")
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        
        print(f"Conectando ao {serverMACAddress}:{port}...")
        s.settimeout(10)  # timeout de conexão aumentado
        s.connect((serverMACAddress, port))
        s.settimeout(0.3)  # timeout curto para não travar recv
        print(f"✓ Conectado com sucesso!")

        while True:
            text = input("Digite mensagem (quit para sair): ")
            if text.lower() == "quit":
                break
            
            try:
                s.send(text.encode('utf-8'))
                print(f"Enviado: {text}")
            except Exception as e:
                print(f"Erro ao enviar: {e}")
                break

            try:
                data = s.recv(1024)
                if data:
                    print("Recebido:", data.decode('utf-8'))
            except socket.timeout:
                # Se não receber nada, continua normalmente
                print("(sem resposta)")
            except Exception as e:
                print(f"Erro ao receber: {e}")

    except Exception as e:
        print("Erro na conexão Bluetooth:", e)
        print("\nPossíveis soluções:")
        print("1. Verifique se o ESP32 está em modo de pareamento")
        print("2. Tente executar como administrador: sudo python3 test.py")
        print("3. Pareie o dispositivo primeiro nas configurações do sistema")
        print("4. Verifique se a porta RFCOMM (1) está correta")
    finally:
        try:
            s.close()
            print("Conexão fechada")
        except:
            pass
else:
    print("\n✗ Não foi possível encontrar o dispositivo Bluetooth alvo.")
    print("\nDispositivos encontrados:")
    for addr, name in nearby_devices:
        print(f"  - {name} ({addr})")
    
    print("\nDicas para solução:")
    print("1. Verifique se o ESP32 está ligado e em modo de pareamento")
    print("2. Execute como administrador: sudo python3 test.py")
    print("3. Verifique se o nome do dispositivo está correto (atual: 'esp32')")
    print("4. Tente parejar o dispositivo primeiro nas configurações do sistema")
    print("5. Reinicie o serviço Bluetooth: sudo systemctl restart bluetooth")
