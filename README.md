# MKW41Z_BLE_Environmental_Sensor
This project uses MKW41Z to collect environmental data and send it out via BLE

Version 0.1 
Is the very first working Demo uses Kinetis BLE Toolbox from apple Appstore or Google Play as the mobile App.
The MCU program adopts the frdmkw41z_wireless_examples_bluetooth_health_thermometer_freertos demo, and hence In the Kinetis BLE Toolbox, we need to choose Health Thermometer demo. 
After powering up, press SW4 on FRDM-KW41Z EVM board, and click connect from moble App. The temperature is showed as "Temperature", and relative humidity is showed as the battery level "Status" at the bottom of the App UI.


kw41z_environmental_sensor.py is the Python script runs on a raspberrypi 3 model B. Uses Instapush for cloud service provider. https://instapush.im/