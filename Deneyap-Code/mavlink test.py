# Bağlantı Kurmafrom pymavlink
import mavutilmaster = mavutil.mavlink_connection('COM13',57600)master.wait_heartbeat()print("Bağlantı kuruldu!")
# GPS Verisi Okuma
while True: msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
if msg: print(f"Lat: {msg.lat/1e7}, Lon: {msg.lon/1e7}")
# Araca Basit Komut Gönderme (Örnek: Motorları Çalıştırma)
master.mav.command_long_send( master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
