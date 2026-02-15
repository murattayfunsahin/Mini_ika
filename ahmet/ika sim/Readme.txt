İKA Simülasyon (Windows) — Kontrol + Mixing + Ramp + Telemetri
==============================================================

Bu klasörde 3 adet Python dosyası vardır:
1) vehicle_server.py      -> “Araç” tarafı (komut alır, motor komutu üretir, telemetri gönderir)
2) pc_client.py           -> “Kontrol istasyonu” (WASD ile komut üretir ve gönderir)
3) telemetry_viewer.py    -> Telemetri izleyici (araç durumunu ve link yaşını gösterir)

Amaç
-----
Raspberry Pi 4 gelmeden önce PC üzerinde uçtan uca sistemi doğrulamak:
- UDP komut kanalı (PC -> Vehicle)
- Failsafe (timeout -> STOP)
- E-STOP (acil durdurma)
- Mixing (throttle/steer -> left/right)
- Deadband (titreme/vızıltı azaltma)
- Ramp (yumuşak hızlanma/fren)
- Telemetri (Vehicle -> PC, link_age_ms dahil)

Gereksinimler
-------------
- Windows 10/11
- Python 3.10+ (önerilir)
- Ek kütüphane gerekmiyor (socket, json, time, msvcrt standart)

Klasör Yapısı
-------------
D:\ika_sim\
  vehicle_server.py
  pc_client.py
  telemetry_viewer.py
  README.txt

Çalıştırma Sırası (3 CMD penceresi)
-----------------------------------
1) CMD-1 (Araç/Vehicle):
   D:
   cd \ika_sim
   python vehicle_server.py

2) CMD-2 (Telemetri):
   D:
   cd \ika_sim
   python telemetry_viewer.py

3) CMD-3 (Kontrol/Client):
   D:
   cd \ika_sim
   python pc_client.py

Kontroller (pc_client.py)
-------------------------
- W : throttle +0.10  (ileri)
- S : throttle -0.10  (geri)
- A : steer -0.10     (sol)
- D : steer +0.10     (sağ)
- X : throttle=0, steer=0 (sıfırla)
- Q : E-STOP (estop=1)
- E : E-STOP reset (estop=0)
- ESC : client çıkış

Beklenen Davranışlar (Test Senaryoları)
---------------------------------------
1) W’ye bas:
   - telemetry_viewer: thr artar, L/R (actual) pozitif artar.
   - vehicle_server: PWM değerleri kademeli artar (ramp).

2) A veya D’ye bas:
   - telemetry_viewer: steer değişir, L/R ayrışır.
   - Örn: D (sağ) -> left artar, right azalır.

3) Q (E-STOP):
   - state FAULT_ESTOP olur.
   - hedefler 0’a çekilir, actual ramp ile 0’a iner.
   - telemetry_viewer satırında <<< ALARM görünür.

4) ESC ile pc_client kapat:
   - vehicle_server komut alamaz.
   - 200 ms sonra state FAULT_TIMEOUT olur.
   - telemetry_viewer telemetri akmaya devam eder; age_ms 200, 300, 500... diye büyür.
   - <<< ALARM görünür.

Neden UDP?
----------
Komut ve telemetri gibi gerçek zamanlı akışlarda UDP düşük gecikme sağlar.
Kaybolan paketler mümkündür; bu yüzden:
- seq numarası
- timeout failsafe
- estop
gibi mekanizmalar zorunludur.

Pi 4 Gelince Ne Değişecek?
--------------------------
Şu an her şey “localhost (127.0.0.1)” üzerinde çalışıyor.

Pi 4 gelince:
- pc_client.py içindeki
  HOST = "127.0.0.1"
  satırını Pi’nin IP adresi ile değiştir:

  HOST = "192.168.x.y"

vehicle_server.py Pi üzerinde çalışacak (ve PORT=5005 dinleyecek).

Not:
- Telemetri de Pi’den PC’ye geleceği için:
  vehicle_server.py içinde TELEM_IP = "127.0.0.1" satırını PC’nin IP adresi yapmanız gerekir.
  (Şimdilik simülasyonda aynı makinede olduğumuz için 127.0.0.1)

Sık Karşılaşılan Sorunlar
-------------------------
1) “Address already in use”
   - Aynı portu kullanan eski bir süreç açıktır.
   - Çalışan pencerelerde Ctrl+C ile kapat.
   - Gerekirse tüm CMD’leri kapatıp tekrar aç.

2) Telemetri gelmiyor
   - telemetry_viewer.py doğru portta mı? (6006)
   - vehicle_server.py TELEM_PORT 6006 mı?
   - Aynı PC’de mi test ediyorsun? (simülasyon için evet)
   - Üç programın da çalıştığından emin ol.

3) ESC’ye basınca durmuyor
   - ESC sadece pc_client’ı kapatır.
   - vehicle_server sonsuz döngüdür; durdurmak için CMD’de Ctrl+C bas.

4) “Python bulunamadı”
   - Python kurulu değil veya PATH’e ekli değil.
   - python --version ile kontrol et.

Geliştirme Notları (İleri Aşama)
--------------------------------
- Kamera akışı ayrı kanal olmalı (RTSP/WebRTC/UDP ayrı port)
- Komut/telemetri protokolü “type/ver” ile sürümlenmiştir.
- Motor sürme gerçek donanımda GPIO/PWM driver ile değiştirilecektir.
- Deadband ve ramp parametreleri saha testine göre ayarlanmalıdır.

İletişim / Ekip
---------------
Kontrol & Haberleşme: Kaan + Ahmet
Bu repo/sim, TEKNOFEST İKA için kontrol mimarisinin temelini oluşturur.