#!/usr/bin/env python3
"""
Visualizador de Telemetria em Tempo Real - ESP32 NeuroBeep
Com gráficos interativos usando matplotlib
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import asyncio
from datetime import datetime
from bleak import BleakClient, BleakScanner
from collections import deque
import re

# Matplotlib para gráficos
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation

SERVICE_UUID = "8f3b6fcf-6d12-437f-8b68-9a3494fbe656"
CHAR_UUID    = "d5593e6b-3328-493a-b3c9-9814683d8e40"

class TelemetryParser:
    """Parser para dados de telemetria do ESP32"""
    
    @staticmethod
    def parse(data_str):
        """Parse string de telemetria para dicionário"""
        result = {}
        
        # Pattern: NOME:valor
        pattern = r'([A-Z_]+):([-+]?[0-9]*\.?[0-9]+)'
        matches = re.findall(pattern, data_str)
        
        for key, value in matches:
            try:
                # Tenta converter para float
                if '.' in value:
                    result[key] = float(value)
                else:
                    result[key] = int(value)
            except ValueError:
                result[key] = value
        
        return result

class TelemetryPlotter:
    """Gerenciador de gráficos de telemetria"""
    
    def __init__(self, parent_frame, max_points=100):
        self.max_points = max_points
        
        # Buffers de dados (deque para eficiência)
        self.time_data = deque(maxlen=max_points)
        self.encoder_left = deque(maxlen=max_points)
        self.encoder_right = deque(maxlen=max_points)
        self.vel_left = deque(maxlen=max_points)
        self.vel_right = deque(maxlen=max_points)
        self.odom_x = deque(maxlen=max_points)
        self.odom_y = deque(maxlen=max_points)
        self.odom_theta = deque(maxlen=max_points)
        self.imu_ax = deque(maxlen=max_points)
        self.imu_ay = deque(maxlen=max_points)
        self.imu_az = deque(maxlen=max_points)
        self.pwm_left = deque(maxlen=max_points)
        self.pwm_right = deque(maxlen=max_points)
        
        self.start_time = time.time()
        
        # Cria figura com subplots
        self.fig = Figure(figsize=(12, 8), dpi=80)
        self.fig.patch.set_facecolor('#2b2b2b')
        
        # Grid 3x2 de gráficos
        self.ax1 = self.fig.add_subplot(3, 2, 1)  # Velocidades
        self.ax2 = self.fig.add_subplot(3, 2, 2)  # Encoders
        self.ax3 = self.fig.add_subplot(3, 2, 3)  # Odometria XY
        self.ax4 = self.fig.add_subplot(3, 2, 4)  # Odometria Theta
        self.ax5 = self.fig.add_subplot(3, 2, 5)  # IMU
        self.ax6 = self.fig.add_subplot(3, 2, 6)  # PWM
        
        # Estilo dos gráficos
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.set_facecolor('#1e1e1e')
            ax.tick_params(colors='white', labelsize=8)
            ax.spines['bottom'].set_color('white')
            ax.spines['left'].set_color('white')
            ax.spines['top'].set_color('white')
            ax.spines['right'].set_color('white')
            ax.title.set_color('white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.grid(True, alpha=0.2, color='white')
        
        self.fig.tight_layout(pad=2.0)
        
        # Canvas para integrar com Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, parent_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    def add_data(self, telemetry_dict):
        """Adiciona novos dados aos buffers"""
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        
        # Encoders
        self.encoder_left.append(telemetry_dict.get('ENC_L', 0))
        self.encoder_right.append(telemetry_dict.get('ENC_R', 0))
        
        # Velocidades
        self.vel_left.append(telemetry_dict.get('VEL_L', 0.0))
        self.vel_right.append(telemetry_dict.get('VEL_R', 0.0))
        
        # Odometria
        self.odom_x.append(telemetry_dict.get('ODOM_X', 0.0))
        self.odom_y.append(telemetry_dict.get('ODOM_Y', 0.0))
        self.odom_theta.append(telemetry_dict.get('ODOM_TH', 0.0))
        
        # IMU
        self.imu_ax.append(telemetry_dict.get('IMU_AX', 0.0))
        self.imu_ay.append(telemetry_dict.get('IMU_AY', 0.0))
        self.imu_az.append(telemetry_dict.get('IMU_AZ', 0.0))
        
        # PWM
        self.pwm_left.append(telemetry_dict.get('PWM_L', 0.0))
        self.pwm_right.append(telemetry_dict.get('PWM_R', 0.0))
        
    def update_plots(self):
        """Atualiza todos os gráficos"""
        if len(self.time_data) == 0:
            return
        
        times = list(self.time_data)
        
        # 1. Velocidades
        self.ax1.clear()
        self.ax1.plot(times, list(self.vel_left), 'c-', label='Vel L', linewidth=2)
        self.ax1.plot(times, list(self.vel_right), 'm-', label='Vel R', linewidth=2)
        self.ax1.set_title('Velocidades (m/s)', fontsize=10)
        self.ax1.set_xlabel('Tempo (s)', fontsize=8)
        self.ax1.legend(fontsize=8, loc='upper right')
        self.ax1.grid(True, alpha=0.2, color='white')
        
        # 2. Encoders
        self.ax2.clear()
        self.ax2.plot(times, list(self.encoder_left), 'g-', label='Enc L', linewidth=2)
        self.ax2.plot(times, list(self.encoder_right), 'y-', label='Enc R', linewidth=2)
        self.ax2.set_title('Encoders (pulsos)', fontsize=10)
        self.ax2.set_xlabel('Tempo (s)', fontsize=8)
        self.ax2.legend(fontsize=8, loc='upper right')
        self.ax2.grid(True, alpha=0.2, color='white')
        
        # 3. Odometria XY
        self.ax3.clear()
        if len(self.odom_x) > 1:
            self.ax3.plot(list(self.odom_x), list(self.odom_y), 'r-', linewidth=2)
            self.ax3.plot(list(self.odom_x)[-1], list(self.odom_y)[-1], 'ro', markersize=8)
        self.ax3.set_title('Trajetória (m)', fontsize=10)
        self.ax3.set_xlabel('X (m)', fontsize=8)
        self.ax3.set_ylabel('Y (m)', fontsize=8)
        self.ax3.axis('equal')
        self.ax3.grid(True, alpha=0.2, color='white')
        
        # 4. Odometria Theta
        self.ax4.clear()
        self.ax4.plot(times, list(self.odom_theta), 'b-', linewidth=2)
        self.ax4.set_title('Orientação (rad)', fontsize=10)
        self.ax4.set_xlabel('Tempo (s)', fontsize=8)
        self.ax4.grid(True, alpha=0.2, color='white')
        
        # 5. IMU
        self.ax5.clear()
        self.ax5.plot(times, list(self.imu_ax), 'r-', label='Ax', linewidth=1.5, alpha=0.7)
        self.ax5.plot(times, list(self.imu_ay), 'g-', label='Ay', linewidth=1.5, alpha=0.7)
        self.ax5.plot(times, list(self.imu_az), 'b-', label='Az', linewidth=1.5, alpha=0.7)
        self.ax5.set_title('Aceleração IMU (m/s²)', fontsize=10)
        self.ax5.set_xlabel('Tempo (s)', fontsize=8)
        self.ax5.legend(fontsize=7, loc='upper right')
        self.ax5.grid(True, alpha=0.2, color='white')
        
        # 6. PWM
        self.ax6.clear()
        self.ax6.plot(times, list(self.pwm_left), 'c-', label='PWM L', linewidth=2)
        self.ax6.plot(times, list(self.pwm_right), 'm-', label='PWM R', linewidth=2)
        self.ax6.set_title('PWM Motors', fontsize=10)
        self.ax6.set_xlabel('Tempo (s)', fontsize=8)
        self.ax6.set_ylim(-10, 265)
        self.ax6.legend(fontsize=8, loc='upper right')
        self.ax6.grid(True, alpha=0.2, color='white')
        
        # Aplica estilo novamente (após clear)
        for ax in [self.ax1, self.ax2, self.ax3, self.ax4, self.ax5, self.ax6]:
            ax.set_facecolor('#1e1e1e')
            ax.tick_params(colors='white', labelsize=8)
            ax.title.set_color('white')
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
        
        self.canvas.draw()
        
    def clear_data(self):
        """Limpa todos os dados"""
        self.time_data.clear()
        self.encoder_left.clear()
        self.encoder_right.clear()
        self.vel_left.clear()
        self.vel_right.clear()
        self.odom_x.clear()
        self.odom_y.clear()
        self.odom_theta.clear()
        self.imu_ax.clear()
        self.imu_ay.clear()
        self.imu_az.clear()
        self.pwm_left.clear()
        self.pwm_right.clear()
        self.start_time = time.time()

# Importa a classe original do controller
import sys
sys.path.insert(0, r'c:\Users\Rinaldo\Documents\neurobeep\controle')

# Copia todo o código do esp32_ble_controller.py e adiciona a funcionalidade de gráficos
