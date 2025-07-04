#!/bin/bash

echo "🤖 Neurassist Bluetooth GUI - Instalação e Execução"
echo "=================================================="

# Verificar se Python3 está instalado
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 não encontrado. Instale o Python3 primeiro."
    exit 1
fi

echo "✅ Python3 encontrado"

# Verificar se tkinter está disponível
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📦 Instalando tkinter..."
    sudo apt-get update
    sudo apt-get install -y python3-tk python3-dev libbluetooth-dev
fi

# Verificar se pip está instalado
if ! command -v pip3 &> /dev/null; then
    echo "📦 Instalando pip3..."
    sudo apt-get install -y python3-pip
fi

# Instalar pybluez2
echo "📦 Instalando dependências Bluetooth..."
pip3 install pybluez2

echo ""
echo "🚀 Instalação concluída!"
echo ""
echo "Para executar a interface gráfica:"
echo "  python3 bluetooth_gui.py"
echo ""
echo "Ou execute este script novamente para abrir automaticamente:"
echo "  ./run_gui.sh"
echo ""

# Perguntar se quer executar agora
read -p "Deseja executar a interface agora? (s/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[SsYy]$ ]]; then
    echo "🚀 Iniciando interface gráfica..."
    python3 bluetooth_gui.py
fi
