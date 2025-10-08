#!/usr/bin/env python3
"""
Script para corrigir is_connected() → is_connected no esp32_ble_controller.py
"""

file_path = r"c:\Users\Rinaldo\Documents\neurobeep\controle\esp32_ble_controller.py"

# Lê o arquivo
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Faz as substituições
replacements = [
    ("await self.client.is_connected()", "self.client.is_connected"),
    ("self.client.is_connected()", "self.client.is_connected"),
]

for old, new in replacements:
    content = content.replace(old, new)

# Escreve de volta
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("✅ Arquivo corrigido com sucesso!")
print("\nSubstituições feitas:")
print("- await self.client.is_connected() → self.client.is_connected")
print("- self.client.is_connected() → self.client.is_connected")
