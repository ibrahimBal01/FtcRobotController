# Kod haritası ve denetim sonucu — 27 Ekim 2026

İlk aktarımın 26 dosyası masaüstündeki asıl projede SHA-256 ile doğrulandı: eksik veya farklı dosya yoktu. Bu kontrol kullanım limitinin etkisini varsaymak yerine diskteki gerçek sonucu doğrular.

## Denetimde düzeltilenler

1. Hood hareketi tamamlanmadan yeni hedef geldiğinde eski hareket süresi kaybolabiliyordu. HoodController artık tamamlanmamış hareketi yeni hareket bütçesine ekler. Aynı hedef süreci uzatmaz; küçük deadband içindeki güncelleme fiziksel olarak gönderilmez.
2. Hood servo kalibrasyonu hareket sırasında değiştiğinde veya geçersiz/aynı uç konumları verildiğinde eski “hazır” bilgisi kullanılamaz.
3. Turret homing sonrası encoder oranı, encoder/servo yönü, limit polaritesi veya açı sınırları değişirse referans geçersiz olur; yeniden homing gerekir. Negatif/sonlu olmayan kontrol parametreleri reddedilir.
4. TrackTarget ve ShootOne komutlarının çalışma sırasına bağlı eski hareket verisiyle atışa başlama olasılığı kapatıldı. MotionGate en son yerelleştirme örneğini bir kez işler; birden fazla sorgu hızını sıfır kabul etmez.
5. GamepadEx.getLeftY zaten Y işaretini çeviriyordu. MainTeleOp ikinci ters çevirmeyi kaldırır; X lateral girişi Pedro için sol-pozitif yapılır. Fiziksel teker yönleri gerçek robotta doğrulanmalıdır.
6. DefaultDriveCommand iptal edildiğinde ve SimpleAuto reset edildiğinde hemen durdurma eklendi.
7. Büyük fakat sonlu açı sınırlarında integer sayaç taşmasına yol açabilecek eşdeğer-açı döngüsü sabit zamanlı hesaplamaya çevrildi.

Derleme sonucu: assembleDebug başarılı. Testler: 21 temel + 17 regresyon = 38 test, 0 hata. Bu testler algoritma ve durum makinelerini çalıştırır; gerçek motor/sensör, robot üzerinde atış veya donanımla tüm OpMode akışı test edilmedi.

## Android Studio içinde dosyalar

Java kökü: TeamCode → src → main → java → org.firstinspires.ftc.teamcode.

| Yer | Görev |
|---|---|
| Robot.java | Drive'ı ve isteğe bağlı iki ShotChannel'ı oluşturur; toplu stop sağlar. |
| ShootingConfig.java | Yeni mekanizmaların bütün isim, yön, oran, limit, hedef, PID/P katsayısı ve kalibrasyon ayarları. |
| Constants.java | Önceki kullanıcı ayarları korunmuştur. Eski shooter/feeder alanları yeni kontrolün ayar kaynağı değildir. |
| subsystems/Drive.java | Mevcut Pedro Follower. Tek periodic güncellemesi, global pose doğrulaması, Pinpoint READY, global heading'i değiştirmeyen sürücü referansı. |
| subsystems/Turret.java | CRServo, encoder portu ve manyetik switch donanım katmanı. |
| aiming/TurretController.java | Homing, debounce, açı kontrolü, limitler ve fault mantığı. |
| subsystems/Hood.java | Pozisyon servosuna komut gönderir. |
| aiming/HoodController.java | Açı→servo konumu eşlemesi ve muhafazakâr hareket/bekleme süresi. |
| subsystems/Shooter.java | Her örnek tek flywheel motoru; encoder RPM, P+feedforward, voltaj telafisi, kararlı RPM kontrolü. |
| subsystems/Feeder.java | DISABLED / SERVO / MOTOR seçimi; kapama, darbe tekrar beklemesi, kısa komut zaman aşımı. |
| aiming/ShotChannel.java | Bir top boyutunun turret+hood+shooter+feeder topluluğu. LARGE ve SMALL bağımsızdır. |
| aiming/FieldTargets.java | İttifak ve yükselmiş uç seçimi; karşı ittifak yerine kendi hedefini döndürür. |
| aiming/AimMath.java | Robot/pivot offset dönüşümü, hedef yaw ve mesafe, mekanik sınırdaki eşdeğer açı. |
| aiming/ShotTable.java | Ölçülmüş mesafe–RPM–hood satırları arasında doğrusal enterpolasyon. |
| aiming/MotionGate.java | En son pose örneğinden translasyon/dönüş hızını kontrol eder. |
| aiming/FeedCycle.java | WAITING → OPEN → DONE veya ABORTED tek atış akışı. |

## Komutların görevleri

| Komut | Çalışması |
|---|---|
| HomeTurretCommand | Switch başlangıçta aktifse önce çıkar, sonra düşük güçle minimum uca yaklaşır; debounce sonrası referanslar. |
| SetTurretAngleCommand | Robot gövdesine göre derece hedefi; ulaşınca veya hata varsa biter. |
| SetHoodAngleCommand | Pozisyon servosuna derece hedefi; bekleme modeli tamamlanınca biter. |
| TrackTargetCommand | Sürekli komuttur. Pedro pose→pivot→seçilen ağız→turret açısı; tablodan RPM ve hood; atış hazırlığı. Drive requirement'ı almaz. |
| ShootOneCommand | Feeder requirement'ını alır. TrackTarget hazırken tek pulse; timeout, hedef kaybı veya iptalde kapanır. |
| SetShooterRpmCommand | RPM hedefini bir kere değiştirir, motor subsystem periodic ile düzenlemeye devam eder. |
| SpinUpShooterCommand | RPM hedefi verir, kararlı devre kadar bekler. Kullanırken withTimeout ekleyin; kesilirse motor durur. |
| StopShooterCommand | RPM ve motor gücünü hemen sıfırlar. Flywheel FLOAT nedeniyle mekanik olarak süzülür. |
| FeedCommand | Alan/konum gerektirmeyen bench kalibrasyon darbesi. Yarışma akışında ShootOne kullanılır. |
| commands/drive/* | Mevcut DefaultDrive, FollowPath, HoldPoint. DefaultDrive iptalinde durur. |

Turret, Hood ve Shooter'ı TrackTarget sahiplenir. Bu üçünden birini isteyen homing veya manuel ayar komutu TrackTarget'ı keser. ShootOne yalnızca feeder'ı sahiplenir, böylece takip ve besleme eşzamanlı çalışabilir.

## Çalışma akışı

1. INIT: donanımlar açılır; yeni turretler UNHOMED'dur. Feeder varsayılan DISABLED'dır.
2. Doğru saha başlangıç pozu, ittifak ve yükselmiş uç seçilir.
3. İlgili turret home edilir. Switch minimum uçta varsayılır; orta konumda switch için bu algoritma uygun değildir.
4. Takip başlatılır. İki turret aynı ağız noktasını farklı pivot konumlarından hedefler.
5. Mesafe kalibrasyon tablosu kapsamındaysa ayrı RPM/hood hedefleri çıkarılır.
6. Pinpoint READY, saha/montaj doğrulanmış, turret ve hood hazır, devir kararlı ve robot yeterince yavaşsa READY olur.
7. Tek atış tuşuna basılınca hazır olma beklenir (varsayılan 4 saniye); feeder varsayılan 0.20 saniye açılır, sonra kapanır. Yeni atış için 0.35 saniye feeder beklemesi ve yeniden kararlı devir gerekir.
8. Tahterevalli taraf değiştirince sürücü karşı tarafa geçer ve diğer yükselmiş ucu seçer. Seçim takibi/atışı keser; yeniden takip başlatılır.

Tahterevalli eğimi sensörle izlenmez. Hedefler sadece en yukarıdaki ağız konumlarıdır. Robot hareket ederken nişan güncellenir, fakat hareket halinde atış telafisi yoktur. Diğer tarafa otomatik sürüş rotası eklenmedi.

## OpMode'lar

- Main TeleOp: drive + iki kanal. Tüm turret, hood ve shooter donanım adları INIT sırasında bulunmalıdır; feeder disabled ise onun donanımı gerekmez.
- Shooter Tuner: iki ayrı flywheel motoru; A büyük aç/kapat, B küçük aç/kapat, BACK durdur. Başka mekanizma gerektirmez.
- Shot Calibration (stationary): INIT öncesi LARGE_CHANNEL seçilir. Tek kanal; A home, X manuel RPM/hood/açı ayarlarını uygula, RB tek bench atışı, BACK durdur. Robot sabit tutulur; bu mod saha hareket denetimini bilinçli olarak kullanmaz.
- Simple Auto: önceki örnek sürüş rotası. Robot(hardwareMap) drive-only oluşturur. Atışlı otonom değildir.

Main TeleOp gamepad 2:
X büyük home; Y küçük home; RB büyük takip; LB küçük takip; A büyük atış; B küçük atış.
D-pad sol kırmızı; sağ mavi; aşağı Audience; yukarı Scoring. BACK iki atış mekanizmasını durdurur.
Gamepad 1 LB yalnızca sürücünün field-centric yön referansını sıfırlar; global pose'u bozmaz.

## Şu anda bilinçli olarak eksik/kapalı olanlar

- FIELD_CALIBRATED=false ve START_POSE_CONFIRMED=false.
- İki kanal için mountCalibrated=false.
- Turret ve hood calibrated=false.
- İki shotTable boş.
- Feeder mode=DISABLED ve calibrated=false.

Bu nedenle mevcut varsayılanlarla otomatik top atılmaz. Bunlar limit yüzünden yarım yazılmış kod değil, gerçek robot ölçümleri gerektiren ayarlardır. Flag'leri ölçüm yapmadan true yapmak eksik kalibrasyonu tamamlamaz.

Ölçülecekler: min/max/home açısı; encoder ve servo yönleri; büyük encoderın gerçek mili; hood açı/servo eşlemesi; flywheel gerçek aktarım oranı; atış mesafe–RPM–hood tablosu; feeder son tipi/konumları; doğru saha başlangıç pozu ve iki turret offseti.

CAD'den alınan saha ağız koordinatları başlangıç tahminidir. Başlangıç pozu (0,0,0) gerçek robot konumu olarak kabul edilmemelidir. MainTeleOp şu anda Auto'nun son pozunu otomatik devralmaz; START_* alanları gerçek TeleOp başlangıcıyla eşleşmeli veya ayrıca poz aktarımı/relocalization eklenmelidir.

Pinpoint bağlantı/status ve güncelleme zamanı denetlenir ancak bütün sessiz sensör arızaları tespit edilemez. Hood gerçek açı sensörüyle doğrulanmaz. Top algılama, mekanik çarpışma analizi ve hareketli atış telafisi yoktur. Bunlar kalan robot entegrasyon sınırlarıdır.

## Bağlantı notları

Yeni shooter adları largeShooter/smallShooter'dır; eski shooterLeft/shooterRight isimleri otomatik kullanılmaz.
Turret encoderName varsayılanları frontLeft/backLeft mevcut motor portlarının encoder soketleridir. Bu plan fiziksel kablolama doğrulaması değildir. Aynı porta ikinci motor tanımı eklenmez; turret kodu o motora güç/mod yazmaz. Bu soketlere bağlı sürüş motorları encoder hız/pozisyon kontrol modunda çalıştırılmamalıdır.
CAD oranı büyük son ara mil için 225/80=2.8125, küçük için 186/75=2.48 encoder turu/turret turudur. Encoder turret halkasında ise 1; başka mildeyse o milin oranı girilir.

Ayrıntılı saha ve ilk çalıştırma adımları SHOOTING_SETUP_TR.md içindedir. Yeni atış ayarlarını ShootingConfig.java'da yapın; kalıcı değişiklikleri Dashboard'dan kaynak dosyaya taşıyın.

