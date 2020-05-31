# Lightning Sensor

Lightning sensor using the AS3935 Franklin Lightning Sensor IC.

***

## WiFi

Device connects to WiFI for:

* Publishing MQTT
* OTA

## MQTT

Device published data over MQTT over WiFi to [Adafruit IO](https://io.adafruit.com).

## Data

Device publishes data as a json object.

Events can be

* `ERROR`
* `DISTURBER`
* `LIGHTNING`
* `NOISE`

>Note: Only the `LIGHTNING` event will give a valid distance value. Other events will give a distance of `0`.

Example:

```json
{
  "event":"LIGHTNING",
  "distance":40
}
```

## OTA

Firmware can be updated via OTA by pressing the OTA button. This will enable the device to go into FOTA update mode.

>FOTA update mode lasts for `3` minutes before timeout.
