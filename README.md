SwitchDoc Labs LoRa SolarMAX Source code for the 433MHz Transmitter (Tx) in SolarMAX2

Designed for the Mini Pro Plus Arduino Board from SwitchDoc Labs<BR>
(select Mini Pro 16MHz/5V in Arduino IDE)

May 17, 2021

SOFTWAREVERSION 014 

014 - Full Update for Mini Pro Plus Board - Software works for both SolarMAX2 



This uses Protocol_ID of 8 and 10

Selects Protocol 8 if device has LiPo battery (SolarMAX LiPo)<BR>
Selects Protocol 10 if device has Lead Acid battery (SolarMAX Lead Acid)


This software requires you to install RH_RF95 as an Arduino Library.  You can download it here (it is also included as Grove_RadioHead-master-3.zip).  Install it the Arduino IDE Library

https://github.com/Seeed-Studio/Grove_LoRa_433MHz_and_915MHz_RF/archive/master.zip


Copy these libraries into the Arduino IDE Library:

Jeelib.zip
Time.zip

Then unzip these files in the libraries directory to install.  Restart your Arduino IDE

