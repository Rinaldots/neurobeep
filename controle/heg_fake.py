from flask import Flask, jsonify
import random
import time

app = Flask(__name__)

def gerar_dados_raw_feedback(qtd=200):
    """
    Gera dados no formato:
    [
        [valor, timestamp],
        [valor, timestamp],
        ...
    ]
    """
    dados = []
    ts = int(time.time() * 1000)

    for i in range(qtd):
        # Simula valores entre 400 e 900 (ajuste conforme necessário)
        valor = random.uniform(400, 900)
        dados.append([valor, ts + i])

    return dados


@app.route("/Feedback/RawData/200", methods=["GET"])
def raw_data_200():
    dados = gerar_dados_raw_feedback(200)
    return jsonify(dados)


if __name__ == "__main__":
    print("Servidor mock rodando em http://localhost:5001")
    app.run(host="0.0.0.0", port=5001, debug=False)
