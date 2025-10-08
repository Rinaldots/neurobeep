#!/usr/bin/env python3
"""
Script de Diagnóstico BLE - Testa conexão com ESP32
"""
import asyncio
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
CHAR_UUID = "d5593e6b-3328-493a-b3c9-9814683d8e40"

async def scan_devices():
    print("🔍 Escaneando dispositivos BLE por 10 segundos...")
    devices = await BleakScanner.discover(timeout=10)
    
    print(f"\n📱 Encontrados {len(devices)} dispositivos:")
    for i, d in enumerate(devices, 1):
        print(f"  {i}. {d.name or '(sem nome)'} - {d.address}")
        print(f"     RSSI: {d.rssi if hasattr(d, 'rssi') else 'N/A'}")
    
    return devices

async def test_connection(address):
    print(f"\n🔗 Tentando conectar ao {address}...")
    
    try:
        # Cria cliente sem callback primeiro
        client = BleakClient(address, timeout=20.0)
        
        print("   Conectando...")
        await client.connect()
        
        if client.is_connected:
            print("✅ Conectado com sucesso!")
            
            # Lista serviços
            print("\n📋 Serviços disponíveis:")
            for service in client.services:
                print(f"   Service: {service.uuid}")
                for char in service.characteristics:
                    print(f"      Char: {char.uuid}")
                    print(f"         Props: {char.properties}")
            
            # Verifica se o serviço esperado existe
            print(f"\n🔍 Procurando serviço {SERVICE_UUID}...")
            service = client.services.get_service(SERVICE_UUID)
            if service:
                print("✅ Serviço encontrado!")
                
                print(f"\n🔍 Procurando característica {CHAR_UUID}...")
                char = service.get_characteristic(CHAR_UUID)
                if char:
                    print("✅ Característica encontrada!")
                    print(f"   Propriedades: {char.properties}")
                    
                    # Tenta enviar um comando de teste
                    if "write" in char.properties or "write-without-response" in char.properties:
                        print("\n📤 Enviando comando de teste 'TEST'...")
                        test_data = "TEST".encode("utf-8")
                        await client.write_gatt_char(CHAR_UUID, test_data, response=True)
                        print("✅ Comando enviado com sucesso!")
                        
                        # Aguarda um pouco
                        await asyncio.sleep(2)
                else:
                    print(f"❌ Característica {CHAR_UUID} não encontrada!")
            else:
                print(f"❌ Serviço {SERVICE_UUID} não encontrado!")
            
            # Desconecta
            print("\n🔌 Desconectando...")
            await client.disconnect()
            print("✅ Desconectado com sucesso!")
            
        else:
            print("❌ Falha ao conectar - is_connected() retornou False")
            
    except asyncio.TimeoutError:
        print("❌ Timeout ao tentar conectar (>20s)")
    except Exception as e:
        print(f"❌ Erro na conexão: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()

async def main():
    print("=" * 60)
    print("Script de Diagnóstico BLE - ESP32 Neuro_Robot")
    print("=" * 60)
    
    # Escaneia dispositivos
    devices = await scan_devices()
    
    # Procura por Neuro_Robot
    target_device = None
    for d in devices:
        if d.name and "neuro" in d.name.lower():
            target_device = d
            print(f"\n🎯 Dispositivo ESP32 encontrado: {d.name} ({d.address})")
            break
    
    if not target_device:
        print("\n❌ Dispositivo 'Neuro_Robot' não encontrado!")
        print("\n💡 Dispositivos disponíveis:")
        for d in devices:
            if d.name:
                print(f"   - {d.name} ({d.address})")
        
        # Permite conexão manual
        if devices:
            print("\n⌨️  Digite o número do dispositivo para testar (ou Enter para sair):")
            try:
                choice = input("> ").strip()
                if choice and choice.isdigit():
                    idx = int(choice) - 1
                    if 0 <= idx < len(devices):
                        target_device = devices[idx]
            except:
                pass
    
    if target_device:
        await test_connection(target_device.address)
    else:
        print("\n❌ Nenhum dispositivo selecionado para teste.")
    
    print("\n" + "=" * 60)
    print("Diagnóstico concluído!")
    print("=" * 60)

if __name__ == "__main__":
    asyncio.run(main())
