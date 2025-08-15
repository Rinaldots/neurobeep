import requests, pandas as pd
import numpy as np

def obter_dados_raw_feedback():
    """Função para consumir dados do endpoint de feedback"""
    url = "http://localhost:5001/Feedback/RawData/200"
    
    try:
        resposta = requests.get(url)
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

# Executar a função
if __name__ == "__main__":
    # Lista para armazenar os valores de mu
    valores_mu = []
    
    period_ms = 200
    trial = 0
    trial_limit = int(30 / (period_ms / 1000))
    
    while trial < trial_limit:
        tmp = obter_dados_raw_feedback()
        mu = calc_avg(tmp)
        
        # Armazenar o valor de mu
        valores_mu.append(mu)
        
        print(trial, mu)
        
        if mu > 400:
            trial = trial_limit + 1
            # Hora de enviar 'stop' pelo BT 
        else:
            trial += 1

    print(f"Valores de mu armazenados: {valores_mu}")