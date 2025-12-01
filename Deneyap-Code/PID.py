import numpy as np
import time
import threading
from pymavlink import mavutil
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import queue
from collections import deque


class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-400, 400)):
        """Basit PID Kontrolörü"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits

        # PID değişkenleri
        self.setpoint = 0.0
        self.last_input = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def update(self, current_value):
        """PID çıkışını hesapla"""
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0.001:
            dt = 0.001

        # Hata hesapla
        error = self.setpoint - current_value

        # Proportional
        p_term = self.kp * error

        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative (input'un türevi - derivative kick önleme)
        d_input = (current_value - self.last_input) / dt
        d_term = -self.kd * d_input

        # Toplam çıkış
        output = p_term + i_term + d_term

        # Limitleri uygula
        output = max(self.output_limits[0], min(self.output_limits[1], output))

        # Değerleri sakla
        self.last_input = current_value
        self.last_time = current_time

        return output, p_term, i_term, d_term


class MotorController:
    def __init__(self, mavlink_connection='COM7', baud=115200):
        self.mavlink_connection = mavlink_connection
        self.baud = baud
        self.master = None

        # PID kontrolörü - basit ayarlar
        self.roll_pid = PIDController(
            kp=5.0,
            ki=0.1,
            kd=1.0,
            output_limits=(-400, 400)
        )

        # Motor ayarları
        self.base_pwm = 1500  # Base PWM (nötr nokta)
        self.motor_pwm = 1500

        # Veri saklamak için
        self.data_queue = queue.Queue()
        self.running = True

        # Güncel değerler
        self.current_roll = 0.0
        self.target_roll = 0.0
        self.last_pid_output = 0.0

        # Basit grafik
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        self.time_data = deque(maxlen=200)
        self.roll_data = deque(maxlen=200)
        self.target_data = deque(maxlen=200)
        self.pid_output_data = deque(maxlen=200)
        self.motor_data = deque(maxlen=200)
        self.start_time = time.time()

    def mavlink_reader(self):
        """MAVLink verilerini okuyan thread"""
        try:
            print(f"MAVLink bağlantısı: {self.mavlink_connection} @ {self.baud}")
            self.master = mavutil.mavlink_connection(self.mavlink_connection, baud=self.baud)
            print("✓ MAVLink bağlantısı başarılı!")

            while self.running:
                msg = self.master.recv_match(timeout=1)
                if not msg:
                    continue

                if msg.get_type() == 'AHRS2':
                    # Roll açısını al (radyandan dereceye)
                    roll_degrees = np.degrees(msg.roll)
                    self.current_roll = roll_degrees

                    # PID hesaplama
                    pid_output, p_term, i_term, d_term = self.roll_pid.update(roll_degrees)
                    self.last_pid_output = pid_output

                    # Motor PWM hesaplama - Basit: Base PWM + PID çıkışı
                    self.motor_pwm = self.base_pwm + int(pid_output)

                    # PWM sınırlarını uygula
                    self.motor_pwm = max(1100, min(1900, self.motor_pwm))

                    # Motor komutunu gönder
                    self.send_motor_commands()

                    # Veriyi kaydet
                    self.data_queue.put({
                        'time': time.time() - self.start_time,
                        'roll': roll_degrees,
                        'target': self.target_roll,
                        'pid_output': pid_output,
                        'motor_pwm': self.motor_pwm
                    })

        except Exception as e:
            print(f"MAVLink hatası: {e}")

    def send_motor_commands(self):
        """Motor komutunu gönder"""
        if self.master:
            try:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    int(self.motor_pwm),  # Motor 1
                    0, 0, 0, 0, 0, 0, 0  # Diğer kanallar
                )
            except Exception as e:
                print(f"Motor komut hatası: {e}")

    def update_plot(self, frame):
        """Grafik güncelleme"""
        # Yeni verileri al
        while not self.data_queue.empty():
            try:
                data = self.data_queue.get_nowait()
                self.time_data.append(data['time'])
                self.roll_data.append(data['roll'])
                self.target_data.append(data['target'])
                self.pid_output_data.append(data['pid_output'])
                self.motor_data.append(data['motor_pwm'])
            except queue.Empty:
                break

        if len(self.time_data) == 0:
            return

        # Grafikleri temizle
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()

        times = list(self.time_data)

        # Roll grafiği
        self.ax1.plot(times, list(self.roll_data), 'b-', label='Roll', linewidth=2)
        self.ax1.plot(times, list(self.target_data), 'r--', label='Hedef', linewidth=2)
        self.ax1.set_ylabel('Roll (°)')
        self.ax1.set_title('Roll Kontrolü')
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.3)

        # PID çıkışı
        self.ax2.plot(times, list(self.pid_output_data), 'g-', label='PID Çıkışı', linewidth=2)
        self.ax2.set_ylabel('PID Çıkışı')
        self.ax2.set_title('PID Çıkışı')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.3)
        self.ax2.axhline(y=0, color='k', linestyle='-', alpha=0.3)

        # Motor PWM
        self.ax3.plot(times, list(self.motor_data), 'r-', label='Motor PWM', linewidth=2)
        self.ax3.axhline(y=1500, color='k', linestyle='--', alpha=0.5, label='Base PWM')
        self.ax3.set_ylabel('PWM')
        self.ax3.set_xlabel('Zaman (s)')
        self.ax3.set_title('Motor PWM (Base: 1500 + PID)')
        self.ax3.legend()
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_ylim(1100, 1900)

        plt.tight_layout()

    def set_target_roll(self, target):
        """Hedef roll açısını ayarla"""
        self.roll_pid.setpoint = target
        self.target_roll = target
        print(f"Hedef roll: {target}°")

    def set_pid_gains(self, kp, ki, kd):
        """PID kazançlarını ayarla"""
        self.roll_pid.kp = kp
        self.roll_pid.ki = ki
        self.roll_pid.kd = kd
        print(f"PID: Kp={kp}, Ki={ki}, Kd={kd}")

    def user_interface(self):
        """Basit kullanıcı arayüzü"""
        print("\n" + "=" * 50)
        print("BASIT MOTOR KONTROLÖR KOMUTLARI:")
        print("=" * 50)
        print("r <açı>    : Hedef roll açısını ayarla (örn: r 10)")
        print("p <değer>  : Kp ayarla (örn: p 8.0)")
        print("i <değer>  : Ki ayarla (örn: i 0.5)")
        print("d <değer>  : Kd ayarla (örn: d 2.0)")
        print("stop       : Motoru durdur")
        print("status     : Mevcut durumu göster")
        print("quit       : Çıkış")
        print("=" * 50)
        print("HESAPLAMA: Motor PWM = 1500 + PID_Çıkışı")

        while self.running:
            try:
                command = input("\nKomut: ").strip().lower()

                if command.startswith('r '):
                    target = float(command.split()[1])
                    self.set_target_roll(target)

                elif command.startswith('p '):
                    kp = float(command.split()[1])
                    self.set_pid_gains(kp, self.roll_pid.ki, self.roll_pid.kd)

                elif command.startswith('i '):
                    ki = float(command.split()[1])
                    self.set_pid_gains(self.roll_pid.kp, ki, self.roll_pid.kd)

                elif command.startswith('d '):
                    kd = float(command.split()[1])
                    self.set_pid_gains(self.roll_pid.kp, self.roll_pid.ki, kd)

                elif command == 'stop':
                    self.motor_pwm = 1500
                    print("Motor durduruldu!")

                elif command == 'status':
                    print(f"\nDURUM:")
                    print(f"Roll: {self.current_roll:.2f}° | Hedef: {self.target_roll:.2f}°")
                    print(f"Hata: {self.target_roll - self.current_roll:.2f}°")
                    print(f"PID: Kp={self.roll_pid.kp}, Ki={self.roll_pid.ki}, Kd={self.roll_pid.kd}")
                    print(f"PID Çıkışı: {self.last_pid_output:.1f}")
                    print(f"Motor PWM: {int(self.motor_pwm)} (1500 + {int(self.last_pid_output)})")

                elif command == 'quit':
                    self.stop()
                    break

            except (ValueError, IndexError):
                print("Geçersiz komut!")
            except KeyboardInterrupt:
                self.stop()
                break

    def start_control(self):
        """Kontrolü başlat"""
        print("Motor kontrolörü başlatılıyor...")

        # MAVLink thread
        mavlink_thread = threading.Thread(target=self.mavlink_reader)
        mavlink_thread.daemon = True
        mavlink_thread.start()

        time.sleep(1)

        # Grafik animasyonu
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)

        # Kullanıcı arayüzü thread
        ui_thread = threading.Thread(target=self.user_interface)
        ui_thread.daemon = True
        ui_thread.start()

        plt.show()

    def stop(self):
        """Durdur"""
        self.running = False
        if self.master:
            try:
                self.master.mav.rc_channels_override_send(
                    self.master.target_system,
                    self.master.target_component,
                    1500, 0, 0, 0, 0, 0, 0, 0
                )
            except:
                pass
            self.master.close()
        print("Motor kontrolörü durduruldu.")


def main():
    """Ana fonksiyon"""
    print("BASIT ROLL PID MOTOR KONTROLÖRÜ")
    print("=" * 40)

    com_port = input("COM port (varsayılan COM7): ").strip() or "COM7"
    baud_input = input("Baud rate (varsayılan 115200): ").strip()
    baud_rate = int(baud_input) if baud_input else 115200

    try:
        controller = MotorController(com_port, baud_rate)

        print("\n⚠️  GÜVENLİK:")
        print("- Motor PWM = 1500 + PID_Çıkışı")
        print("- PID çıkışı ±400 sınırlı")
        print("- PWM aralığı: 1100-1900")

        input("\nBaşlatmak için Enter...")

        controller.start_control()

    except KeyboardInterrupt:
        print("\nProgram durduruldu.")
    except Exception as e:
        print(f"Hata: {e}")


if __name__ == "__main__":
    main()