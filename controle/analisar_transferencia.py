import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import os

class MotorTransferFunctionAnalyzer:
    def __init__(self, csv_file):
        """
        Analisador de função de transferência do motor (lado direito)
        Suporta dois formatos de CSV:
          Formato legado: timestamp, datetime, pwm_r, r_vel
          Formato novo:   timestamp, datetime, pwm_right, vel_right
        """
        self.csv_file = csv_file
        self.data = None
        self.transfer_function = None
        # Nomes canônicos usados internamente
        self.col_pwm = 'pwm'
        self.col_vel = 'vel'

    def _map_columns(self, df):
        """Mapear colunas do CSV para nomes canônicos pwm/vel com fallback."""
        cols = df.columns.tolist()
        pwm_col = None
        vel_col = None
        # Possibilidades de nomes
        pwm_candidates = ['pwm_right', 'pwm_r', 'pwm']
        vel_candidates = ['vel_right', 'r_vel', 'vel']
        for c in pwm_candidates:
            if c in df.columns:
                pwm_col = c
                break
        for c in vel_candidates:
            if c in df.columns:
                vel_col = c
                break
        if pwm_col is None or vel_col is None:
            raise ValueError(f"Não encontrei colunas PWM/vel esperadas. Colunas disponíveis: {cols}")
        if pwm_col != 'pwm_right':
            print(f"[AVISO] Usando coluna PWM '{pwm_col}' (não é 'pwm_right')")
        if vel_col != 'vel_right':
            print(f"[AVISO] Usando coluna Vel '{vel_col}' (não é 'vel_right')")
        # Renomear para canônicos temporários
        df = df.rename(columns={pwm_col: self.col_pwm, vel_col: self.col_vel})
        return df

    def load_data(self):
        """Carrega e prepara os dados"""
        try:
            raw = pd.read_csv(self.csv_file)
            self.data = self._map_columns(raw)
            print(f"Dados carregados: {len(self.data)} amostras")
            if 'timestamp' not in self.data.columns:
                raise ValueError("Coluna 'timestamp' ausente no CSV")
            # Converter timestamp para tempo relativo
            self.data['time'] = self.data['timestamp'] - self.data['timestamp'].iloc[0]
            # Calcular período de amostragem médio
            if len(self.data) > 1:
                dt = np.mean(np.diff(self.data['time']))
            else:
                dt = np.nan
            if not np.isnan(dt):
                print(f"Período médio: {dt:.4f} s  |  Freq: {1/dt:.2f} Hz")
            # Estatísticas básicas
            print(f"\nPWM - Min: {self.data[self.col_pwm].min()}, Max: {self.data[self.col_pwm].max()}")
            print(f"Velocidade - Min: {self.data[self.col_vel].min():.4f}, Max: {self.data[self.col_vel].max():.4f}")
            return True
        except Exception as e:
            print(f"Erro ao carregar dados: {e}")
            return False

    def analyze_steady_state(self):
        """Analisa resposta em regime permanente"""
        if self.data is None:
            print("Carregue os dados primeiro")
            return
        pwm_groups = self.data.groupby(self.col_pwm)[self.col_vel]
        steady_state_data = []
        for pwm, vel_series in pwm_groups:
            n_samples = len(vel_series)
            if n_samples < 5:
                continue
            start_idx = int(0.8 * n_samples)
            steady_slice = vel_series.iloc[start_idx:]
            steady_state_data.append({
                'pwm': pwm,
                'velocity': steady_slice.mean(),
                'std': steady_slice.std(),
                'n': n_samples
            })
        if not steady_state_data:
            print("Nenhum grupo PWM suficiente para regime permanente")
            return None
        self.steady_state = pd.DataFrame(steady_state_data).sort_values('pwm').reset_index(drop=True)
        print("\n=== REGIME PERMANENTE ===")
        print(self.steady_state)
        return self.steady_state

    def fit_first_order_model(self):
        """Ajusta modelo de primeira ordem: G(s) = K/(τs + 1)"""
        if self.data is None:
            print("Carregue os dados primeiro")
            return
        pwm_series = self.data[self.col_pwm]
        vel_series = self.data[self.col_vel]
        pwm_diff = np.diff(pwm_series)
        step_indices = np.where(np.abs(pwm_diff) > 10)[0]
        if len(step_indices) == 0:
            print("Nenhum degrau detectado")
            return
        print(f"\n=== DEGRAU ===  (detectados {len(step_indices)})")
        step_idx = step_indices[0]
        pre_step = self.data.iloc[max(0, step_idx-50):step_idx]
        post_step = self.data.iloc[step_idx:min(len(self.data), step_idx+200)].copy()
        pwm_initial = pre_step[self.col_pwm].mean()
        pwm_final = post_step[self.col_pwm].iloc[-50:].mean()
        vel_initial = pre_step[self.col_vel].mean()
        vel_final = post_step[self.col_vel].iloc[-50:].mean()
        delta_pwm = pwm_final - pwm_initial
        delta_vel = vel_final - vel_initial
        if delta_pwm == 0:
            print("Degrau inválido (delta PWM zero)")
            return
        print(f"PWM: {pwm_initial:.1f} -> {pwm_final:.1f}")
        print(f"Vel: {vel_initial:.4f} -> {vel_final:.4f}")
        K = delta_vel / delta_pwm
        print(f"Ganho K: {K:.6f}")
        target_vel = vel_initial + 0.63 * delta_vel
        post_step['time'] = post_step['time'] - post_step['time'].iloc[0]
        try:
            tau_idx = np.where(post_step[self.col_vel] >= target_vel)[0][0]
            tau = post_step['time'].iloc[tau_idx]
            print(f"Tau: {tau:.3f} s")
            print(f"G(s) = {K:.6f} / ({tau:.3f}s + 1)")
            self.transfer_function = {'K': K, 'tau': tau, 'type': 'first_order'}
        except IndexError:
            print("Não atingiu 63% do degrau para estimar tau")
        return post_step, vel_initial, vel_final, target_vel

    def fit_polynomial_model(self, degree=2):
        """Ajusta modelo polinomial PWM -> Velocidade"""
        if self.data is None:
            print("Carregue os dados primeiro")
            return
        steady_state = self.analyze_steady_state()
        if steady_state is None or len(steady_state) < 3:
            print("Dados insuficientes para polinômio")
            return
        pwm_values = steady_state['pwm'].values
        vel_values = steady_state['velocity'].values
        valid_mask = vel_values > 0.001
        pwm_clean = pwm_values[valid_mask]
        vel_clean = vel_values[valid_mask]
        if len(pwm_clean) < 3:
            print("Poucos pontos válidos após limpeza")
            return
        coeffs = np.polyfit(pwm_clean, vel_clean, degree)
        poly_func = np.poly1d(coeffs)
        vel_pred = poly_func(pwm_clean)
        ss_res = np.sum((vel_clean - vel_pred) ** 2)
        ss_tot = np.sum((vel_clean - np.mean(vel_clean)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        print(f"\n=== POLINÔMIO (grau {degree}) ===")
        print(f"Coeficientes: {coeffs}")
        print(f"R²: {r_squared:.4f}")
        eq_terms = []
        for i, coeff in enumerate(coeffs):
            power = degree - i
            if power == 0:
                eq_terms.append(f"{coeff:.6f}")
            elif power == 1:
                eq_terms.append(f"{coeff:.6f}*PWM")
            else:
                eq_terms.append(f"{coeff:.6f}*PWM^{power}")
        print("Equação: Vel = " + ' + '.join(eq_terms))
        self.polynomial_model = {'coeffs': coeffs, 'r_squared': r_squared, 'degree': degree}
        return steady_state, poly_func

    def plot_analysis(self):
        if self.data is None:
            print("Carregue os dados primeiro")
            return
        fig, axes = plt.subplots(2, 2, figsize=(14, 9))
        axes[0, 0].plot(self.data['time'], self.data[self.col_pwm], 'b-', label='PWM')
        ax_twin = axes[0, 0].twinx()
        ax_twin.plot(self.data['time'], self.data[self.col_vel], 'r-', label='Vel')
        axes[0, 0].set_title('PWM & Velocidade (Direito)')
        axes[0, 0].set_xlabel('Tempo (s)')
        axes[0, 0].set_ylabel('PWM')
        ax_twin.set_ylabel('Vel (m/s)')
        axes[0, 0].grid(alpha=0.3)
        axes[0, 1].scatter(self.data[self.col_pwm], self.data[self.col_vel], s=4, alpha=0.5)
        axes[0, 1].set_xlabel('PWM')
        axes[0, 1].set_ylabel('Vel (m/s)')
        axes[0, 1].set_title('Dispersão PWM→Vel')
        axes[0, 1].grid(alpha=0.3)
        try:
            steady_state, poly_func = self.fit_polynomial_model(degree=2)
            if steady_state is not None:
                pwm_range = np.linspace(steady_state['pwm'].min(), steady_state['pwm'].max(), 120)
                vel_model = poly_func(pwm_range)
                axes[1, 0].scatter(steady_state['pwm'], steady_state['velocity'], color='C0', s=40, label='Regime')
                axes[1, 0].plot(pwm_range, vel_model, 'r-', lw=2, label='Polinômio')
                axes[1, 0].set_xlabel('PWM')
                axes[1, 0].set_ylabel('Vel (m/s)')
                axes[1, 0].set_title('Modelo Estático')
                axes[1, 0].legend()
                axes[1, 0].grid(alpha=0.3)
        except Exception as e:
            axes[1, 0].text(0.5, 0.5, f'Erro polinômio\n{e}', ha='center', va='center', transform=axes[1, 0].transAxes)
        plt.tight_layout()
        plt.show()

    def generate_report(self):
        if self.data is None:
            print("Carregue os dados primeiro")
            return
        print("\n" + "="*48)
        print("RELATÓRIO - MOTOR DIREITO")
        print("="*48)
        print(f"Arquivo: {self.csv_file}")
        print(f"Amostras: {len(self.data)} | Duração: {self.data['time'].max():.2f} s")
        steady_state = self.analyze_steady_state()
        self.fit_first_order_model()
        self.fit_polynomial_model(degree=2)
        print("="*48)

def main():
    # Procurar formatos novos e legados
    csv_files = [f for f in os.listdir('.') if f.endswith('.csv') and ('motor_r' in f or 'right' in f or 'motor' in f)]
    if not csv_files:
        print("Nenhum arquivo CSV de motor encontrado - gerando dataset sintético para demonstração.")
        t = np.linspace(0, 8, 800)
        pwm_seq = np.piecewise(t, [t < 2, (t >= 2) & (t < 4), (t >= 4) & (t < 6), t >= 6], [0, 80, 140, 200])
        # Sistema de 1ª ordem sintético (K=0.0009 m/s por PWM, tau=0.5s)
        K_true = 0.0009
        tau_true = 0.5
        vel = np.zeros_like(t)
        for i in range(1, len(t)):
            dt = t[i] - t[i-1]
            vel[i] = vel[i-1] + dt * (-(vel[i-1]) / tau_true + (K_true * pwm_seq[i]) / tau_true)
        vel += np.random.normal(0, 0.002, size=vel.shape)
        df = pd.DataFrame({
            'timestamp': t + 1_000_000,  # deslocamento
            'datetime': pd.Timestamp.now(),
            'pwm_right': pwm_seq.astype(int),
            'vel_right': vel
        })
        synth_file = 'synthetic_motor_right.csv'
        df.to_csv(synth_file, index=False)
        print(f"Dataset sintético salvo em {synth_file}")
        csv_files = [synth_file]
    latest_file = max(csv_files, key=os.path.getctime)
    print(f"Analisando arquivo: {latest_file}")
    analyzer = MotorTransferFunctionAnalyzer(latest_file)
    if analyzer.load_data():
        analyzer.generate_report()
        try:
            resp = input("\nGerar gráficos? (s/n): ")
            if resp.lower() in ('s', 'sim', 'y', 'yes'):
                analyzer.plot_analysis()
        except Exception:
            print("Sem interação para gráficos")

if __name__ == "__main__":
    main()
