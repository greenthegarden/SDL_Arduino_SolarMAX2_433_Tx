SwitchDoc Labs SolarMAX2 Source code for the 433MHz Transmitter (Tx) in SolarMAX2

Designed for the Mini Pro Plus Arduino Board from SwitchDoc Labs<BR>
(select Mini Pro 16MHz/5V in Arduino IDE)

September 7, 2021

SOFTWAREVERSION 015 

015 - Minor bug fixes and tuning

014 - Full Update for Mini Pro Plus Board - Software works for both SolarMAX2 



This uses Protocol_ID of 8 and 10

Selects Protocol 8 if device has LiPo battery (SolarMAX2 LiPo)<BR>
Selects Protocol 10 if device has Lead Acid battery (SolarMAX2 Lead Acid)


This software requires you to install RH_RF95 as an Arduino Library.  You can download it here (it is also included as Grove_RadioHead-master-3.zip).  Install it the Arduino IDE Library

https://github.com/Seeed-Studio/Grove_LoRa_433MHz_and_915MHz_RF/archive/master.zip


Copy these libraries into the Arduino IDE Library:

Jeelib.zip
Time.zip

Then unzip these files in the libraries directory to install.  Restart your Arduino IDE

