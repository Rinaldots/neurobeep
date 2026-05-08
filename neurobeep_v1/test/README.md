# Neurassist Bluetooth Terminal - Interface Gráfica

Uma interface gráfica moderna para comunicação Bluetooth com o ESP32 do projeto Neurassist.

## Características

- **Interface Gráfica Intuitiva**: Desenvolvida em tkinter
- **Dois Terminais Separados**: 
  - Terminal de mensagens recebidas
  - Terminal de histórico de envios
- **Escaneamento Automático**: Detecta dispositivos Bluetooth disponíveis
- **Conexão Automática**: Conecta-se facilmente ao ESP32
- **Comunicação Bidirecional**: Envia e recebe mensagens em tempo real
- **Logs com Timestamp**: Todas as mensagens são marcadas com horário

## Instalação

### Pré-requisitos

1. **Python 3.x** instalado
2. **tkinter** (normalmente incluído com Python)
3. **Bluetooth** habilitado no sistema

### Instalação das Dependências

```bash
# Instalar dependências do sistema (Ubuntu/Debian)
sudo apt-get update
sudo apt-get install python3-tk python3-dev libbluetooth-dev

# Instalar dependências Python
pip3 install -r requirements.txt
```

## Como Usar

### Método 1: Script de Inicialização (Recomendado)

```bash
./run_gui.sh
```

### Método 2: Execução Direta

```bash
python3 bluetooth_gui.py
```

### Método 3: Com Privilégios de Administrador (se necessário)

```bash
sudo python3 bluetooth_gui.py
```

## Instruções de Uso

### 1. Inicialização
- Execute o programa
- O sistema verificará automaticamente se o Bluetooth está funcionando

### 2. Conectar ao ESP32
1. Clique em **"Escanear Dispositivos"**
2. Aguarde o escaneamento completar (pode demorar alguns segundos)
3. Selecione o dispositivo ESP32 na lista
4. Clique em **"Conectar"**

### 3. Comunicação
- **Enviar mensagens**: Digite no campo de entrada e pressione Enter ou clique "Enviar"
- **Receber mensagens**: Aparecem automaticamente no terminal de mensagens recebidas
- **Histórico**: Todas as mensagens enviadas ficam registradas no terminal de envios

### 4. Desconectar
- Clique em **"Desconectar"** ou feche a janela

## Interface

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Neurassist Bluetooth Terminal                     │
├─────────────────────┬───────────────────────────────────────────────┤
│       Controle      │              Comunicação                      │
│                     │                                               │
│ Status: Conectado   │  Mensagens Recebidas:                        │
│                     │ ┌───────────────────────────────────────────┐ │
│ [Escanear Disp.]    │ │ [10:30:15] 📱 Sensor: 25.3°C             │ │
│                     │ │ [10:30:20] 📱 Motor: Ligado               │ │
│ Dispositivos:       │ │ [10:30:25] 📱 Bateria: 85%               │ │
│ ┌─────────────────┐ │ └───────────────────────────────────────────┘ │
│ │ ESP32 (XX:XX:XX)│ │                                               │
│ │ Device 2        │ │  Enviar Mensagem:                            │
│ │ Device 3        │ │ ┌─────────────────────────────────┐ [Enviar] │
│ └─────────────────┘ │ │ Digite sua mensagem...          │          │
│                     │ └─────────────────────────────────┘          │
│ [Conectar][Descon.] │                                               │
│                     │  Histórico de Envios:                        │
│                     │ ┌───────────────────────────────────────────┐ │
│                     │ │ [10:29:50] 📤 ligar_motor                 │ │
│                     │ │ [10:30:10] 📤 status                      │ │
│                     │ └───────────────────────────────────────────┘ │
└─────────────────────┴───────────────────────────────────────────────┘
```

## Recursos Técnicos

- **Threading**: Comunicação assíncrona sem travamentos
- **Queue**: Comunicação thread-safe entre threads
- **Reconexão**: Detecção automática de perda de conexão
- **Tratamento de Erros**: Mensagens de erro informativas
- **Logging**: Registro completo de todas as ações

## Solução de Problemas

### Erro: "Adaptador Bluetooth não funciona"
```bash
# Reiniciar serviço Bluetooth
sudo systemctl restart bluetooth

# Verificar status
sudo systemctl status bluetooth
```

### Erro: "Permission denied"
```bash
# Executar como administrador
sudo python3 bluetooth_gui.py
```

### Erro: "No module named 'bluetooth'"
```bash
# Instalar dependências
sudo apt-get install python3-dev libbluetooth-dev
pip3 install pybluez
```

### ESP32 não aparece na lista
1. Verifique se o ESP32 está ligado
2. Verifique se está em modo de pareamento
3. Tente parejar primeiro nas configurações do sistema
4. Execute como administrador

## Arquivos

- `bluetooth_gui.py` - Interface gráfica principal
- `test.py` - Versão original em linha de comando
- `requirements.txt` - Dependências Python
- `run_gui.sh` - Script de inicialização
- `README.md` - Esta documentação

## Desenvolvido para

Projeto Neurassist - Assistente Neural com Comunicação Bluetooth
