IKA Vision Brain – README (Pi 4 / Camera Module 3)
==================================================

Amaç
-----
Bu modül Raspberry Pi 4 üzerindeki Camera Module 3 görüntüsünden "throttle" (hız) ve "steer" (yön) komutları üretir.
Komutlar log’a yazılır ve ileride Pico’ya seri port üzerinden gönderilmeye hazırdır.

Bu README; kodun uçtan uca çalışma mantığını, debug ekranlarını ve üretilen log formatını anlatır.

Klasör Yapısı
-------------
~/ika/
  config/
    vision_config.json    -> tüm ayarlar (fps, HSV maske, smoothing, vb.)
  logs/
    vision.log            -> üretilen V satırları (append ile birikir)
  src/
    vision_brain.py       -> ana program

Çıktı Formatı (Log / Konsol)
----------------------------
Her frame (yaklaşık fps kadar) bir satır üretir:

  V,<throttle>,<steer>,<conf>

- "V" : Vision mesajı (ileride başka tipler eklenebilir)
- throttle : 0..1 arası hız isteği
    0.00 = dur
    0.80 = maksimum (throttle_max ile clamp)
- steer : -1..+1 arası yön isteği
    -1.00 = sola tam kır
     0.00 = düz
    +1.00 = sağa tam kır
- conf : 0..1 arası güven (hedefi ne kadar “net” görüyorum)
    0.00 = hedef yok (fail-safe devreye girebilir)
    1.00 = hedef çok net

Örnek:
  V,0.376,-0.925,0.293
  -> throttle %37.6, sola sert (steer -0.925), güven 0.293

Önemli Not (Log)
----------------
Log dosyası "append" modda açıldığı için her çalıştırmada sona eklenir.
En güncel satırları görmek için:

  tail -n 50 ~/ika/logs/vision.log

Debug Ekranları (Pencereler)
----------------------------
debug_preview=true ise iki pencere açılır:

1) "Vision Debug" penceresi:
   - Yeşil yatay çizgi: ROI başlangıcı (roi_y)
     (Görüntünün üst kısmı yok sayılır, alt kısım analiz edilir.)
   - Turuncu/sarı yatay çizgi (band_top): ROI içindeki "band" sınırı
     (Band: ROI’nun alt kısmından seçilen ince şerit. Asıl karar buradan alınır.)
   - Kırmızı dikey çizgi: Algoritmanın bulduğu hedef merkez (cx)
     Steer hesabı bu çizginin konumuna göre yapılır.
   - Üstteki yazı:
       thr=...  steer=...  conf=...  lost=...
     thr/steer/conf: o frame’in komutları
     lost: art arda hedef kayıp frame sayısı

2) "Mask" penceresi:
   - Siyah/Beyaz maske görüntüsü
   - Beyaz görünen pikseller "hedef" olarak kabul edilir
   - Burada amaç: sadece zemindeki çizgi/şerit gibi hedefin beyaz çıkması
     İnsan/duvar/ışık parlaması gibi şeylerin mümkün olduğunca siyah kalması

Kodun Çalışma Akışı (Pipeline)
------------------------------
Program saniyede yaklaşık fps kez döner ve şu adımları uygular:

A) Kamera açılışı
   - Picamera2 başlatılır
   - 640x480 RGB888 formatında frame alınır

B) ROI (Region Of Interest) seçimi
   - Görüntünün üst kısmı atılır, alt kısım analiz edilir
   - ROI başlangıcı:
       y0 = int(h * roi_y)
     roi = frame[y0:, :]
   - Sebep: Yol/zemin çoğunlukla görüntünün altındadır; gürültü azalır.

C) Band seçimi (ROI içinde alt şerit)
   - ROI’nun tamamı yerine, alt tarafındaki daha küçük bir band kullanılır:
       bh = int(roi_height ' band_h)
       band = roi[-bh:, :]
   - Sebep: Kamera yanlışlıkla yukarıyı görse bile karar zemine yakın banddan çıkar.
     Bu sahada stabiliteyi ciddi artırır.

D) HSV dönüşümü
   - RGB yerine HSV kullanılır:
       hsv = cv2.cvtColor(band, cv2.COLOR_RGB2HSV)
   - Sebep: Beyaz/şerit gibi hedefleri "düşük S (saturation) + yüksek V (value)"
     ile daha temiz ayıklamak.

E) Maske üretimi (inRange)
   - Beyaz-ish alanları seçmek için:
       lower = [0, 0, v_min]
       upper = [180, s_max, 255]
       bw = inRange(hsv, lower, upper)
   - Parametreler:
       v_min : parlaklık alt sınırı (yüksek olursa sadece daha parlak alanlar seçilir)
       s_max : doygunluk üst sınırı (düşük olursa renkli şeyler elenir)

F) Gürültü temizleme (morphology)
   - Küçük noktaları silmek ve delikleri kapatmak için:
       bw = OPEN (küçük gürültüyü azaltır)
       bw = CLOSE (hedef içindeki boşlukları doldurur)
   - morph_k: kernel boyutu (5 genelde başlangıç için iyi)

G) En büyük bileşeni (blob) seçme
   - Mask içinde birden çok beyaz bölge olabilir.
   - connectedComponentsWithStats ile:
     - tüm bileşenler bulunur
     - en büyük alanlı bileşen seçilir
   - Sebep: Hedef genelde en baskın beyaz bölgedir; küçük parazitler elenir.

H) Hedef merkez (cx) ve alan
   - Seçilen en büyük bileşenin centroid’i alınır:
       cx
   - Alan:
       area_px
   - Minimum alan kontrolü:
       area_px < min_area_px ise hedef yok sayılır -> conf=0 (fail-safe tetiklenebilir)

I) Steer hesaplama (-1..+1)
   - Merkeze göre normalize:
       steer = (cx - w/2) / (w/2)
       clamp(-1..+1)

J) Confidence (0..1)
   - Alan büyüdükçe güven artar:
       conf = area_px / conf_area_norm
       clamp(0..1)
   - conf_area_norm ayarı büyürse conf daha yavaş 1’e çıkar.

K) Throttle (0..1)
   - Temel kural:
       throttle = 0.2 + 0.6*conf
   - Sonra "sert dönüşte yavaşla" kuralı:
       throttle = throttle * (1 - steer_slow_k * abs(steer))
     steer_slow_k yükseldikçe sert dönüşlerde daha çok yavaşlar.
   - En son limitler:
       throttle_min .. throttle_max aralığına clamp (throttle > 0 ise)

L) Fail-safe (hedef kaybolursa stop)
   - conf çok düşükse lost_cnt artar
   - lost_cnt >= conf_stop_frames olursa:
       throttle=0
       steer=0
       conf=0
   - Sebep: hedef yokken araç kontrolsüz hızlanmasın.

M) Smoothing (EMA)
   - steer zıplamasını azaltmak için:
       steer_ema = (1-alpha)*steer_ema + alpha*steer
   - alpha ~ 0.2-0.3 iyi başlangıçtır (yükselirse daha hızlı tepki, düşerse daha yumuşak)

N) Log yazma
   - Her frame:
       V,throttle,steer,conf
     satırı konsola basılır ve log’a yazılır.

Parametreler (vision_config.json)
---------------------------------
fps              : döngü hızı (frame/s)
debug_preview    : debug pencereleri açık/kapalı
log_path         : log dosyası yolu

roi_y            : ROI başlangıcı (0.55 -> alt %45’i analiz)
band_h           : ROI içindeki band yüksekliği oranı (0.22 gibi)

v_min            : HSV V alt eşiği (parlaklık)
s_max            : HSV S üst eşiği (renksizlik)
morph_k          : morphology kernel boyutu

min_area_px      : en küçük kabul edilen hedef alanı
conf_area_norm   : conf normalizasyonu

ema_alpha        : steer EMA smoothing katsayısı
conf_stop_frames : kaç frame hedef yoksa stop
throttle_min     : minimum throttle (throttle>0 iken)
throttle_max     : maksimum throttle
steer_slow_k     : dönüşte yavaşlama katsayısı

Test Etme (Doğru Senaryo)
-------------------------
- Kamera masaya/zemine baksın (yukarı bakarsa insan/duvar gibi şeyleri hedef sanabilir)
- Zeminde beyaz bant/şerit/kağıt ile hedef oluştur
- Mask penceresinde sadece hedef beyaz çıkmalı
- Vision Debug penceresinde kırmızı çizgi hedefin ortasına yakın olmalı
- Hedef kaybolunca lost artmalı ve stop devreye girmeli

Çalıştırma
----------
1) venv aktif et:
   source ~/ika/.venv/bin/activate

2) Programı çalıştır:
   python ~/ika/src/vision_brain.py

3) Çıkış:
   - Debug penceresinde 'q' ile çık
   - veya terminalde CTRL+C

Sık Karşılaşılan Sorunlar (Encoding)
------------------------------------
Eğer:
  SyntaxError: Non-UTF-8 code starting with ...
görürsen:
- Dosyaya UTF-8 dışı karakter girmiştir (genelde editör / Türkçe karakter / kopyala-yapıştır).
- En güvenli çözüm: dosyayı terminalden here-doc (cat <<'PY') ile yeniden yazmak.
- Bu projede kodu mümkünse bu yöntemle güncellemek önerilir.

Sonraki Adım (Next Part)
------------------------
Bu vision modülünden çıkan "V,throttle,steer,conf" satırları Pico’ya seri port ile gönderilecek.
Pico tarafında:
- MPU6050 + HC-SR04 verileri alınacak
- Pi’dan gelen vision komutuyla birleştirilecek
- BTS7960B sürücüler üzerinden 4 motor kontrol edilecek

Vision Configuration – vision_config.json
========================================

Dosya Yolu
----------
~/ika/config/vision_config.json

Örnek Config
------------
{
  "fps": 20,
  "log_path": "~/ika/logs/vision.log",
  "debug_preview": true,

  "ema_alpha": 0.25,
  "conf_stop_frames": 8,
  "throttle_min": 0.12,
  "throttle_max": 0.80,

  "roi_y": 0.55,
  "band_h": 0.22,

  "v_min": 185,
  "s_max": 65,
  "morph_k": 5,
  "min_area_px": 2500,
  "conf_area_norm": 15000,

  "steer_slow_k": 0.6
}

Parametre Açıklamaları
---------------------

GENEL AYARLAR
-------------
fps
- Vision döngüsünün saniyedeki tekrar sayısı
- Örn: 20 → saniyede ~20 frame
- Çok yükselirse CPU yükü artar

log_path
- Vision çıktılarının yazıldığı dosya
- Format: V,throttle,steer,conf
- Pico seri protokolüyle birebir uyumlu

debug_preview
- true  → Vision Debug + Mask pencereleri açılır
- false → headless (sadece log üretir)

SMOOTHING & FAIL-SAFE
--------------------
ema_alpha
- Steer için EMA (Exponential Moving Average) katsayısı
- 0.2–0.3 → yumuşak ve stabil
- Büyürse daha hızlı ama daha zıplayan direksiyon

conf_stop_frames
- Kaç frame üst üste hedef kaybolursa stop edileceği
- Örn: 8 → ~0.4 saniye (fps=20 iken)

THROTTLE (HIZ) SINIRLARI
-----------------------
throttle_min
- Araç hareket halindeyken izin verilen minimum hız
- Çok küçük olursa motorlar titreyebilir

throttle_max
- Maksimum hız sınırı
- Vision conf=1 olsa bile bu değerin üstüne çıkmaz

ROI ve BAND AYARLARI
-------------------
roi_y
- Görüntünün üstten ne kadarının atılacağı
- 0.55 → üst %55 çöpe, alt %45 analiz edilir
- Kamera ne kadar aşağı bakıyorsa bu değer o kadar yükseltilebilir

band_h
- ROI içindeki analiz bandının yüksekliği
- 0.22 → ROI’nun sadece alt %22’lik kısmı kullanılır
- Amaç: sadece zemine yakın veriye bakmak

HSV MASKE AYARLARI
-----------------
v_min
- HSV Value (parlaklık) alt sınırı
- Yükseldikçe sadece daha parlak alanlar hedef olur
- Çok düşük → her şey beyaz çıkar
- Çok yüksek → hedef kaçabilir

s_max
- HSV Saturation (renk doygunluğu) üst sınırı
- Düşük olursa renkli objeler (cilt, duvar) elenir
- Çok düşük → beyaz hedef bile kaçabilir

MORPHOLOGY (GÜRÜLTÜ TEMİZLEME)
------------------------------
morph_k
- Morphology kernel boyutu
- 3–5 → hafif gürültü
- 7+ → agresif temizlik (ince çizgiler kaybolabilir)

ALAN & CONF HESABI
------------------
min_area_px
- Hedef sayılması için gereken minimum beyaz alan (pixel)
- Küçük parazitleri hedef sanmamak için kullanılır

conf_area_norm
- Alanın confidence’a normalize edildiği referans değer
- Alan = conf_area_norm → conf ≈ 1.0
- Büyürse conf daha yavaş yükselir

DÖNÜŞTE YAVAŞLAMA
----------------
steer_slow_k
- Sert dönüşlerde hızın ne kadar düşeceğini belirler
- throttle = throttle * (1 - steer_slow_k * |steer|)
- 0.0 → dönüşte hız düşmez
- 0.6 → gerçek araç hissi
- 1.0 → çok agresif yavaşlama

TUNING İPUÇLARI
---------------
- Masada test ederken:
  v_min ↑, s_max ↓, band_h ↓
- Gerçek zeminde:
  min_area_px ↑, conf_area_norm ↑
- Araç çok zıplıyorsa:
  ema_alpha ↓
- Dönüşlerde savruluyorsa:
  steer_slow_k ↑
- Araç çok geç duruyorsa:
  conf_stop_frames ↓


Vision Log – vision.log README
==============================

Amaç
-----
vision.log dosyası Vision Brain’in her frame’de ürettiği kararların
zaman içinde kaydını tutar. Bu dosya sistemin “kara kutusu” (blackbox)
gibidir ve şu amaçlarla kullanılır:

- Vision algoritmasının doğru çalışıp çalışmadığını görmek
- HSV, ROI, band, throttle ve steer ayarlarını (tuning) yapmak
- Hangi durumda neden durduğunu veya döndüğünü analiz etmek
- Pico tarafına gidecek komutların geçmişini incelemek

Dosya Yolu
----------
~/ika/logs/vision.log

Dosya Davranışı
---------------
- Dosya "append" modda açılır.
- Program her çalıştırıldığında eski kayıtlar silinmez, sona eklenir.
- Uzun süreli testlerde log büyür; bu istenen bir davranıştır.

Gerekirse dosya manuel olarak sıfırlanabilir:
  : > ~/ika/logs/vision.log

Satır Formatı
-------------
Her satır tek bir frame’e karşılık gelir.

  V,<throttle>,<steer>,<conf>

Alanlar:

V
- Mesaj tipi: Vision komutu
- İleride başka tipler eklenebilir (örn: T=Telemetry)

throttle (0.000 – 1.000)
- Hız isteği
- 0.000 → dur
- throttle_max (örn 0.80) → izin verilen maksimum hız
- conf arttıkça throttle artar
- Sert dönüşlerde throttle otomatik düşürülür

steer (-1.000 – +1.000)
- Yön isteği
- -1.000 → sola tam kır
-  0.000 → düz
- +1.000 → sağa tam kır
- EMA (Exponential Moving Average) ile yumuşatılmıştır

conf (0.000 – 1.000)
- Vision algoritmasının “hedefi ne kadar net gördüğü”
- 0.000 → hedef yok
- 1.000 → hedef çok net
- Beyaz alanın büyüklüğüne göre hesaplanır

Örnek Satırlar
--------------
V,0.000,0.000,0.000
- Hedef yok / fail-safe aktif
- Araç durmalı

V,0.312,0.947,0.187
- Düşük-orta hız
- Sağa sert dönüş
- Güven düşük

V,0.800,-0.163,1.000
- Maksimum hız
- Hafif sola dönüş
- Hedef çok net

Fail-Safe Kayıtları
-------------------
Aşağıdaki satır çok sık görülebilir ve NORMALDİR:

  V,0.000,0.000,0.000

Bu durumlar için üretilir:
- Mask içinde yeterli beyaz alan yoksa
- Alan min_area_px altındaysa
- conf üst üste conf_stop_frames kadar düşük kaldıysa

Amaç:
- Araç kontrolsüz hızlanmasın
- Görüntü kaybolduğunda güvenli şekilde dursun

Log Okuma ve Analiz Komutları
-----------------------------
En son 30 satırı gör:
  tail -n 30 ~/ika/logs/vision.log

Canlı izleme (program çalışırken):
  tail -f ~/ika/logs/vision.log

Sadece stop anlarını bul:
  grep "V,0.000,0.000,0.000" -n ~/ika/logs/vision.log

Belirli bir hız üstünü filtrele (örn throttle > 0.5):
  awk -F',' '$2 > 0.5 {print}' ~/ika/logs/vision.log

Tuning İçin Log Nasıl Kullanılır?
---------------------------------
HSV / Mask Ayarı:
- conf sürekli 1.000 ise → maske fazla geniş
  → v_min ↑ veya s_max ↓
- conf hep 0.000 ise → maske fazla dar
  → v_min ↓ veya s_max ↑

Steer Davranışı:
- steer hızlı zıplıyorsa → ema_alpha ↓
- dönüşler geç kalıyorsa → ema_alpha ↑

Hız Davranışı:
- araç virajda savruluyorsa → steer_slow_k ↑
- araç çok yavaş kalıyorsa → steer_slow_k ↓ veya throttle_max ↑

Fail-Safe:
- çok sık duruyorsa:
  → min_area_px ↓
  → conf_stop_frames ↑
- geç duruyorsa:
  → conf_stop_frames ↓

Pico Entegrasyonu ile İlişkisi
------------------------------
vision.log’daki satırlar Pico’ya gönderilecek seri veriyle birebir aynıdır.
Yani:

  Vision Brain → log’a yazar
  Vision Brain → aynı satırı Pico’ya gönderir

Bu sayede:
- Pico tarafında görülen davranış log’dan birebir izlenebilir
- Sorun çıktığında Pi log’u ile Pico davranışı karşılaştırılabilir

Özet
----
vision.log:
- Vision sisteminin karar geçmişidir
- Tuning için ana referanstır
- Fail-safe ve güvenlik davranışlarını doğrular
- Pico entegrasyonu için test ve debug aracıdır


