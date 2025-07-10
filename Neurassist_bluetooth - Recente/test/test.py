import bluetooth
import socket
import time
import asyncio

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

def is_connection_alive(socket_obj):
    """Verifica se a conexão Bluetooth ainda está ativa"""
    try:
        # Envia um comando de teste (apenas tenta enviar dados sem bloquear)
        socket_obj.settimeout(0.1)
        socket_obj.send(b'')  # Envia um pacote vazio para testar a conexão
        return True
    except socket.timeout:
        return True  # Timeout é normal, conexão provavelmente OK
    except:
        return False  # Qualquer outro erro indica conexão perdida
    finally:
        socket_obj.settimeout(0.5)  # Restaura timeout padrão

# Verificação inicial
print("=== Diagnóstico Bluetooth ===")
if not check_bluetooth_system():
    exit(1)

target_name = "neuro_esp32"
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

def connect_bluetooth(address, port=1, timeout=10):
    """Conecta ao dispositivo Bluetooth e retorna o socket"""
    try:
        print(f"Criando socket Bluetooth...")
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.settimeout(timeout)
        print(f"Conectando ao {address}:{port}...")
        s.connect((address, port))
        s.settimeout(0.5)  # timeout curto para recv
        print(f"✓ Conectado com sucesso!")
        return s
    except Exception as e:
        print(f"Erro na conexão: {e}")
        return None

def auto_reconnect(address, max_attempts=5, delay=3):
    """Tenta reconectar automaticamente com estratégia progressiva"""
    for attempt in range(max_attempts):
        print(f"Tentativa de reconexão {attempt + 1}/{max_attempts}...")
        
        # Delay progressivo: 3s, 5s, 7s, 10s, 15s
        if attempt > 0:
            wait_time = min(delay + (attempt * 2), 15)
            print(f"Aguardando {wait_time} segundos antes da próxima tentativa...")
            time.sleep(wait_time)
        
        s = connect_bluetooth(address)
        if s:
            print("✓ Reconexão bem-sucedida!")
            return s
        else:
            print(f"✗ Tentativa {attempt + 1} falhou")
    
    print("✗ Falha na reconexão automática após todas as tentativas.")
    return None

if target_address is not None:
    print(f"✓ Tentando conectar ao dispositivo: {target_address}")
    
    serverMACAddress = target_address
    port = 1
    s = None
    
    # Conexão inicial
    s = connect_bluetooth(serverMACAddress, port)
    
    if s:
        try:
            last_heartbeat = time.time()
            heartbeat_interval = 30  # Verifica conexão a cada 30 segundos
            
            while True:
                try:
                    # Verifica periodicamente se a conexão ainda está ativa
                    current_time = time.time()
                    if current_time - last_heartbeat > heartbeat_interval:
                        if not is_connection_alive(s):
                            print("⚠ Conexão perdida detectada, tentando reconectar...")
                            s.close()
                            s = auto_reconnect(serverMACAddress)
                            if not s:
                                break
                        last_heartbeat = current_time
                    
                    text = input("Digite mensagem (quit para sair, reconectar para forçar reconexão, status para verificar conexão): ")
                    
                    if text.lower() == "quit":
                        break
                    elif text.lower() == "reconectar":
                        print("🔄 Forçando reconexão...")
                        s.close()
                        s = auto_reconnect(serverMACAddress)
                        if not s:
                            break
                        continue
                    elif text.lower() == "status":
                        alive = is_connection_alive(s)
                        print(f"Status da conexão: {'✓ Ativa' if alive else '✗ Inativa'}")
                        if not alive:
                            print("Tentando reconectar...")
                            s.close()
                            s = auto_reconnect(serverMACAddress)
                            if not s:
                                break
                        continue
                    
                    # Limpa mensagens pendentes do ESP32
                    print("📨 Recebendo mensagens do ESP32...")
                    try:
                        while True:
                            data = s.recv(1024)
                            if data:
                                print("📱 Serial ESP32:", data.decode('utf-8').strip())
                            else:
                                break
                    except socket.timeout:
                        pass
                    except Exception as e:
                        print(f"⚠ Erro ao receber dados: {e}")
                        # Tenta reconectar
                        print("🔄 Tentando reconectar automaticamente...")
                        s.close()
                        s = auto_reconnect(serverMACAddress)
                        if not s:
                            break
                        continue
                    
                    # Envia mensagem
                    try:
                        s.send(text.encode('utf-8'))
                        print(f"📤 Enviado: {text}")
                    except Exception as e:
                        print(f"⚠ Erro ao enviar: {e}")
                        # Tenta reconectar
                        print("🔄 Tentando reconectar automaticamente...")
                        s.close()
                        s = auto_reconnect(serverMACAddress)
                        if not s:
                            break
                        continue

                    async def async_recv(s, loop):
                        try:
                            data = await loop.run_in_executor(None, s.recv, 1024)
                            if data:
                                print("📥 Recebido:", data.decode('utf-8'))
                        except socket.timeout:
                            print("⏱ (sem resposta)")
                        except Exception as e:
                            print(f"⚠ Erro ao receber resposta: {e}")
                            # Não reconecta aqui, pode ser só timeout normal

                    # Dentro do loop principal, substitua por:
                    loop = asyncio.get_event_loop()
                    await async_recv(s, loop)
                        
                except KeyboardInterrupt:
                    print("\n⚠ Interrompido pelo usuário")
                    break
                except Exception as e:
                    print(f"⚠ Erro geral: {e}")
                    print("🔄 Tentando reconectar automaticamente...")
                    if s:
                        s.close()
                    s = auto_reconnect(serverMACAddress)
                    if not s:
                        break
                    
        finally:
            if s:
                try:
                    s.close()
                    print("Conexão fechada")
                except:
                    pass
    else:
        print("Não foi possível estabelecer conexão inicial.")
        print("\nPossíveis soluções:")
        print("1. Verifique se o ESP32 está em modo de pareamento")
        print("2. Tente executar como administrador: sudo python3 test.py")
        print("3. Pareie o dispositivo primeiro nas configurações do sistema")
        print("4. Verifique se a porta RFCOMM (1) está correta")
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
