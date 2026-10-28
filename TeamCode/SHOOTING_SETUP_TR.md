# İki turret ve ayarlanabilir hood — 27 Ekim 2026

Bu değişiklik mevcut FTCLib/Pedro 3.0.1 yapısına eklenmiştir. Sürüş katsayıları, Pinpoint offsetleri ve kullanıcı tarafından düzenlenmiş Constants.java korunur. Yeni atış ayarlarının kaynağı ShootingConfig.java'dır; eski Constants.Shooter/Feeder alanları yeni mekanizmaları yönetmez.

## Davranış

- Her top boyutu için bağımsız Turret, Hood, Shooter ve Feeder.
- Axon sürekli dönüş + REV Through Bore ile açı kontrolü. Manyetik switch ile referanslama, yazılımsal sınırlar, switch debounce, homing timeout, encoder sıçraması/hareketsizliği denetimi.
- Referans switch'i minimum açı ucunda varsayılır. Mıknatıs orta referans konumundaysa bu homing algoritmasını mekanik yerleşime göre değiştirmeden kullanmayın.
- Hedef, seçilen ittifakın seçilen **yukarıdaki ağız noktasıdır**. Sensörle tahterevalli durumu algılanmaz; sürücü Audience/Scoring ucunu seçer.
- Pedro global konumu ve robot üzerindeki iki ayrı turret pivot offseti ile hedef yaw ve yatay mesafe her döngüde yeniden hesaplanır.
- Hood ve flywheel için ayrı ölçülmüş mesafe–RPM–hood tablosu kullanılır. Hood pozisyon servosudur; “hazır” bilgisi ölçülmüş açı değil, hareket mesafesine bağlı bekleme süresidir.
- Hedefe dönüş robot hareket ederken devam eder. **Hareket halinde atış telafisi yoktur:** varsayılan hız sınırları 2 in/s ve 5 deg/s üzerinde feeder açılmaz.
- Bir tuş basımı bir sınırlı feeder darbesidir; hazırlık için bekleme zaman aşımına uğrar. RPM'nin top çıkışından sonra düşmesi darbeyi kesmez; nişan/hood/pozisyon hazırlığı kaybolursa darbe kesilir.
- Hedef/ittifak değişimi bekleyen atışı ve takibi durdurur; yeniden takip başlatılır.
- Yol planlama bu değişikliğe eklenmedi. Sahanın karşı tarafına sürücü veya mevcut otonom komutlarıyla geçilir.

## CAD bulguları ve oranlar

Robot (1).step 347 yaprak parça örneği; önceki Robot.step 359 örnek içeriyordu. Bu adetler BOM değil, STEP montajındaki geometrik yaprak örnekleridir. Yeni dosyada büyük shooter da mevcut, turret aktarma yerleri değişmiş.

Diş sayıları CAD parça adlarından, eşleşmeler geometri merkezlerinden çıkarıldı; fiziksel mekanizmada doğrulayın.

| Kanal | Servo aktarımı | Servo turu / turret turu | Son ara mil encoder turu / turret turu |
|---|---|---:|---:|
| Büyük | 74 → 37; aynı milde 80 → 225 | 1.40625 | 2.8125 |
| Küçük | 74 → 37; aynı milde 75 → 186 | 1.24 | 2.48 |

Yeni CAD'de görülen tek Through Bore, küçük turretin 75 dişli ara mili çevresindedir. Büyük encoderın yeri henüz doğrulanmadı. LARGE için 2.8125 yalnızca 80 dişli ara miline encoder bağlanacağı varsayımıdır. Doğrudan turret halkasına bağlarsanız oran 1 olur. Eski 1.5/3 oranlarını bu yeni CAD için kullanmayın.

REV Through Bore 8192 count/encoder turu kabul edilir:
turret açısı = home açısı + encoderSign × (count-homeCount) × 360 / (8192 × encoderTurnsPerTurretTurn).
Servo oranı bu formüle ayrıca girmez.

## Saha ve robot koordinatları

Saha STEP'inden seçilen **yukarı konumdaki ön ağız çerçevesinin bounding-envelope merkezi** başlangıç nişan noktası olarak alındı. Bu, ölçülmüş en iyi atış noktası veya ağzın alan ağırlıklı merkezi değildir.

- Red Audience CAD yaklaşık (-323.638, -398.559, 1486.575) mm.
- Blue Scoring CAD yaklaşık (+324.062, +398.559, 1486.575) mm.
- Diğer iki yükselmiş uç Y ekseninde yansıtılarak tahmin edildi.
- Kod saha dönüşümü: X=72+CAD_X/25.4; Y=72+CAD_Y/25.4. CAD Z/25.4 yalnızca yükseklik metadatasıdır.
- Audience -Y, Scoring +Y tarafındadır. Başlangıç konumu/heading ve otonom yollar bu eksenlerle aynı olmalıdır; farklı saha çerçevesi kullanıyorsanız tüm hedefleri ve giriş tarafı kontrolünü birlikte dönüştürün.
- Kod yalnızca ağız giriş tarafında atışa izin verir. Engeller için balistik çarpışma kontrolü veya görüş doğrulaması yoktur.
- Tablo, sabit yükselmiş ağız yüksekliğine özel ölçülür; Z değiştirerek otomatik balistik yeniden hesaplama yapılmaz.

Robot montaj offsetleri için geçici referans: CAD teker merkezlerinin ortası (225,130) mm; ileri=-CAD_Y, sol=-CAD_X.
Büyük pivot yaklaşık (128.5,79.847), küçük pivot (332.5,196.507) mm. Pedro yerelleştirme referansınız bu noktadan farklıysa offsetleri düzeltin. Robotun sıfır turret açısındaki atış yönünü yawOffsetDeg ile tanımlayın.
muzzleForwardInches, turret pivotundan çıkış noktasına atış yönündeki radyal uzaklıktır. Yanal namlu offseti bu modelde yoktur; büyükse nişan geometrisine eklenmelidir.

## Donanım adları

| Görev | Büyük | Küçük |
|---|---|---|
| CRServo | largeTurret | smallTurret |
| Manyetik DigitalChannel | largeTurretHome | smallTurretHome |
| Hood Servo | largeHood | smallHood |
| Flywheel DcMotorEx | largeShooter | smallShooter |
| Opsiyonel feeder Servo/DcMotorEx | largeFeeder | smallFeeder |
| Through Bore'un bağlı olduğu mevcut motor portu | frontLeft | backLeft |

Encoder portu isimleri birer **planlanan kablolama seçeneğidir**; mevcut bağlantı olduğunu varsaymayın. Through Bore kablosunu ilgili sürüş motorunun kullanılmayan encoder soketine takın; motorun güç kablosu sürüş motoruna bağlı kalır. HardwareMap'te aynı porta ikinci motor adı oluşturmayın. Turret bu motor nesnesinden yalnızca encoder okur; güç/mod/reset yazmaz. Pinpoint odometri sürüş motor encoderlarını kullanmıyor. Bu portları RUN_USING_ENCODER / RUN_TO_POSITION ile sürüş için kullanmayın. Encoder yönü SDK motor yönünden de etkilenir; yönü kontrol edin. Flywheel encoderlarını turret için paylaşmayın.

CRServo ve Servo nesnelerinin FTC HardwareMap'e nasıl sunulduğunu kontrol edin. CAD'deki REV Servo Hub için bağlantı/adlandırma sürücü desteği bu kodun dışında kalır; standart FTC Servo/CRServo arayüzü beklenir.

Feeder varsayılan DISABLED: donanım gerektirmez, atış yapamaz. Mekanik kesinleşince mode=SERVO veya MOTOR, konum/güç/zaman ve calibrated ayarlarını girin. Mod/isim/yön/oran değişiminden sonra OpMode'u yeniden INIT edin.

## İlk çalıştırma ve kalibrasyon

1. HardwareMap adları, Axon CR modu, encoder soketleri ve switch polaritesini doğrulayın. Encoder/servo pozitif yönleri CCW turret açısı artırmalı. homePower düşük tutulur.
2. Her kanal için gerçek minDeg/maxDeg, switch'in minimum uç konumu, encoder oranı/yönü ve servoSign ayarlanır; ardından turret.calibrated=true.
3. Hood'un gerçek atış açısı ↔ servo konum eşleşmesini ölçün; minDeg/maxDeg ve positionAtMin/Max ayarlayın. Motor dişli oranı bu iki uç kalibrasyona dahildir. Hood.calibrated=true.
4. Shot Calibration (stationary) OpMode'unda LARGE_CHANNEL'i INIT öncesinde seçin. A=home, X=manuel RPM/hood/turret kontrolünü başlat, RB=bir kalibrasyon atışı, BACK=hepsini durdur. Dashboard RPM, HOOD_DEG, TURRET_DEG canlı değişir. Feeder calibrated=false/disabled iken top bırakılmaz. Robotu fiziksel olarak sabit tutun; bu bench OpMode hız/alan denetimi yapmaz.
5. Her top için gerçek ağız yüksekliğine farklı mesafelerden atıp uygun RPM ve hood açısını kaydedin. Örnek sözdizimi: new double[][] { {mesafe1, rpm1, hood1}, {mesafe2, rpm2, hood2} }. **Kodda uydurma atış tablosu yoktur.** En az iki artan mesafe satırı gerekir; tablo dışında extrapolasyon yoktur.
6. Flywheel 6000 RPM goBILDA, motor encoderı 28 tick ve 1:1 başlangıç kabulüdür. Yeni CAD'de kasnaklar bulunduğu için gerçek motorTurnsPerWheelTurn'ü doğrulayın. maxRpm çıkış flywheel limitidir; motor oranı değişirse güncelleyin. Motor doğru yönde dönerken getVelocity() pozitif olmalıdır.
7. Hedef ağızları, CAD→saha eksenini, başlangıç pozunu ve robot montaj offsetlerini ölçün. FIELD_CALIBRATED, START_POSE_CONFIRMED, kanal.mountCalibrated ancak bundan sonra true yapılır.
8. Pinpoint READY olmadan, ilk saha pozu doğrulanmadan, homing yapılmadan veya açı/mesafe sınırı dışındayken otomatik feeder açılmaz.
9. Minimum ve maksimum açı, hatalı/kopuk encoder, switch takılı/aktif, BACK ve STOP davranışlarını tezgâhta doğrulayın. Yazılım testleri fiziksel çarpışma, mekanik sıkışma ve gerçek sensör kablolamasını doğrulamaz.

## Main TeleOp kumandası

Gamepad 1: mevcut sürüş; LB saha sürüş referansını sıfırlar. Global Pedro heading değiştirilmez.
Gamepad 2:

| Tuş | İşlev |
|---|---|
| X / Y | Büyük / küçük turret home |
| RB / LB | Büyük / küçük hedef takibini ve shooter'ı başlat |
| A / B | Büyük / küçük tek atış isteği |
| D-pad sol / sağ | Kırmızı / mavi ittifak |
| D-pad aşağı / yukarı | Audience / Scoring yükselmiş uç |
| BACK | İki atış mekanizmasını ve bekleyen atışları durdur |

İttifak varsayılan RED, uç AUDIENCE. Seçimler otomatik tahterevalli algılama değildir.
STOP'ta servo dönüş güçleri, shooterlar, feederlar ve drivetrain hemen durdurulur. Hood mevcut mekanik pozisyonunu tutar.

## Otonom kullanım

Robot(hardwareMap) drive-only'dir; mevcut SimpleAuto yeni atış donanımı istemez.
Atışlı otonom için Robot(hardwareMap,true) oluşturun:
- drive.setPose(...) ardından doğrulanmış poz için drive.confirmFieldPose(true).
- targets.select(Alliance.RED, End.AUDIENCE).
- Her turret için HomeTurretCommand (aynı anda ancak mekanik çarpışma olmadığı doğrulandıysa).
- TrackTargetCommand'i schedule edin; bu komut sürekli çalışır, sıradaki SequentialCommandGroup adımını bekletmek için kullanmayın.
- Takiple eşzamanlı ShootOneCommand çalıştırın. Drive'ı requirement olarak almazlar.
- Rutin sonunda takipleri iptal edin ve robot.stop() çağırın. Örnek SimpleAuto'nun yolu değiştirilmedi.

Eski ShooterTuner artık iki tek-motor shooter'ı ayrı ayrı ayarlar. FeedCommand yalnızca sınırlı bench kalibrasyon darbesidir; yarışmada ShootOneCommand kullanılır. SpinUpShooterCommand kesilirse motor hemen durur.

## Doğrulama

Android Studio'nun JBR'si ve projenin Gradle wrapper'ı ile assembleDebug ve testDebugUnitTest çalıştırılır. İlk 21 temel test (ek regresyonlarla toplam 38 test): montaj offsetinin robot heading ile dönmesi, açı sarımı ve mekanik sınırlar, tablo enterpolasyonu/reddi, homing/debounce, stuck switch timeout, encoder oranı/yönü, stall, beklenmedik switch, döngü gecikmesi, atış pulse/timeout/aim kaybı ve ittifak seçimi.

Pinpoint READY ve yazılım güncelleme zamanını kontrol etmek her tür sessiz sensör arızasını tespit edemez; gerçek robotta yerelleştirme doğrulaması gerekir. Hareketli atış telafisi ve top algılama bu sürümde yoktur.

Kaynaklar:
- Pedro 3.0.1 kaynakları ve https://pedropathing.com/docs/pathing/guide/teleop-usage
- FTC SDK 12.0.0 GoBildaPinpointDriver kaynakları
- https://www.revrobotics.com/content/docs/REV-11-1271-DS.pdf


27 Ekim ek denetimi: CODE_GUIDE_TR.md dosyasında düzeltilen hatalar ve dosya haritası bulunur.

