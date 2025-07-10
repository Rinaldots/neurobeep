#!/bin/bash

# Script de inicialização para a interface Bluetooth GUI
# Neurassist Bluetooth Terminal

echo "=== Neurassist Bluetooth Terminal ==="
echo "Iniciando interface gráfica..."

# Verifica se o Python está instalado
if ! command -v python3 &> /dev/null; then
    echo "Erro: Python3 não está instalado!"
    exit 1
fi

# Verifica se o tkinter está disponível
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Erro: tkinter não está disponível!"
    echo "Instale com: sudo apt-get install python3-tk"
    exit 1
fi

# Verifica se o pybluez está instalado
python3 -c "import bluetooth" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Instalando dependências..."
    pip3 install -r requirements.txt
    
    # Se ainda não funcionar, tenta instalar dependências do sistema
    if [ $? -ne 0 ]; then
        echo "Instalando dependências do sistema..."
        sudo apt-get update
        sudo apt-get install -y python3-dev libbluetooth-dev
        pip3 install pybluez
    fi
fi

# Executa a interface gráfica
echo "Iniciando interface..."
python3 bluetooth_gui.py
