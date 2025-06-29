import bluetooth
import socket

target_name = "esp32"
target_address = None
nearby_devices = bluetooth.discover_devices(lookup_names=True)
print("Dispositivos encontrados:", nearby_devices)

for btaddr, btname in nearby_devices:
    if target_name == btname:
        target_address = btaddr
        break

if target_address is not None:
    print(f"Encontrado dispositivo {target_name} com endereço {target_address}")

    serverMACAddress = target_address
    port = 1

    try:
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        s.connect((serverMACAddress, port))
        s.settimeout(0.3)  # timeout curto para não travar recv
        print(f"Conectado ao {target_name}")

        while True:
            text = input("Digite mensagem (quit para sair): ")
            if text.lower() == "quit":
                break
            s.send(text.encode('utf-8'))

            try:
                data = s.recv(1024)
                if data:
                    print("Recebido:", data.decode('utf-8'))
            except socket.timeout:
                # Se não receber nada, continua normalmente
                pass

    except Exception as e:
        print("Erro na conexão Bluetooth:", e)
    finally:
        s.close()
else:
    print("Não foi possível encontrar o dispositivo Bluetooth alvo.")
