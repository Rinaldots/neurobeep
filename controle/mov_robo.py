import bluetooth
import socket
import requests, pandas as pd
import numpy as np

def obter_dados_raw_feedback():
    """Função para consumir dados do endpoint de feedback"""
    url = "http://localhost:5001/Feedback/RawData/200"
    
    try:
        resposta = requests.get(url, timeout=(0.1,0.1))
        resposta.raise_for_status()  # Garante que o código de status seja 200 (OK)

        dados = resposta.json()  # Transforma o conteúdo da resposta em JSON
        
        return dados

    except requests.exceptions.RequestException as e:
        print(f"Erro ao fazer requisição: {e}")
        return None
    except ValueError:
        print("Erro ao tentar converter a resposta para JSON.")
        return None

def calc_avg (tmp):
    df = pd.DataFrame(tmp)
    array_original=df.values
    x = np.array([sublista[0] for sublista in array_original])
    mu=np.mean(x)
    return (mu)


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

        text = 'S'
        s.send(text.encode('utf-8'))

        try:
            data = s.recv(1024)
            if data:
                print("Recebido:", data.decode('utf-8'))
        except socket.timeout:
            # Se não receber nada, continua normalmente
            pass

        while True:
            data = pd.DataFrame()
            result=[]
            period_ms=10000
            trial = 0
            trial_limit=int(30/(period_ms/1000))
            while trial<trial_limit:
                tmp=obter_dados_raw_feedback()
                mu = calc_avg (tmp)
                print (trial,mu)
                if mu>540:
                    trial = trial_limit+1
                    text = 'P'
                else:
                    trial +=1
                    text = 'S'
            
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
