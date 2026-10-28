# Basit intake

Intake, topu robotun içine alan tek DC motordur. Feeder'dan bağımsızdır.

- Robot Configuration içinde motoru `intake` adıyla tanımlayın. Encoder kablosu gerekmez.
- `Main TeleOp` INIT olduğunda motor kapalıdır.
- START sonrasında Gamepad 1 RB'ye bir kez basınca açılır; tekrar basınca kapanır. Tuşu bırakınca çalışmaya devam eder.
- Gamepad 2 BACK, intake'ı ve atış sistemlerini durdurur. Driver Station STOP sürüş dahil programı durdurur.
- Hedef/ittifak seçimi intake'ı kapatmaz. Intake, atışa hazır olma koşullarından bağımsızdır.
- Ekranda `Intake ON/OFF` görünür.

Ayarlar `IntakeConfig.java` içindedir. `POWER=0.5` motorun güç isteğidir; sabit RPM anlamına gelmez. İlk denemeyi topsuz yapın. Yön yanlışsa STOP yapıp `REVERSED` değerini değiştirin ve yeniden INIT edin. `MOTOR_NAME` değişikliği de yeniden INIT gerektirir. Dashboard üzerinden POWER değiştirilebilir; geçerli aralık 0'dan büyük, 1'e eşittir veya küçüktür. Geçersiz değer motoru kapatır ve yeniden açma isteği gerektirir.

`subsystems/Intake.java`: start(), stop(), toggle(), isRunning(). MainTeleOp tuşu FTCLib InstantCommand ile subsystem'i kullanır. Intake yalnızca MainTeleOp tarafından oluşturulur; mevcut otonom ve atış test programları ek intake donanımı istemez. Otomatik top algılama, ters çalıştırma, sıkışma algılama ve feeder bağlantısı eklenmedi.
