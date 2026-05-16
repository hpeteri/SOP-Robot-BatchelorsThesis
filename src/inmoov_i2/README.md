# inmoov_i2

InMoov i2-pään ja -silmien sensorisolmut. Tuottaa raakadataa fyysisistä antureista (kamera) uuden data-vetoisen arkkitehtuurin mukaisesti.

## Rakenne

```
inmoov_i2/
├── i2eyes/       # Silmien webcam-sensori
└── i2head/       # (Tulossa) Pään servo-konfiguraatio
```

## i2eyes

### I2eWebcamNode

Webcam-sensorisolmu. Julkaisee kameraa dataa perception-paketin käytettäväksi.

**Julkaisijat:**
- `topic_name` (`sensor_msgs/Image`) – raaka kamerakuva (oletus: `/i2e_webcam`)

### I2eCv2Webcam

Ohut OpenCV `VideoCapture`-kääre. Tarjoaa `is_valid()`- ja `close()`-metodit.

### I2eWebcamSensor

`SensorBase`-toteutus, joka lukee webcam-kuvan ja julkaisee sen ROS Image -viestinä.

### I2eWebcamNode

`SensorNodeBase`-toteutus, joka käynnistää webcam-sensorin ajastimella.

## ROS2-parametrit

| Parametri | Tyyppi | Oletus | Kuvaus |
|-----------|--------|--------|--------|
| `topic_name` | string | `/i2e_webcam` | Julkaisun topic |
| `camera_index` | int | 0 | Kameraindeksi |
| `camera_width` | int | 1280 | Kuvan leveys |
| `camera_height` | int | 960 | Kuvan korkeus |
| `camera_fps` | int | 30 | Kuvanopeus |

## Huomioitavaa

- i2head-alipaketti on varattu i2Head-moduulin servokonfiguraatiolle (YAML-pohjainen moduulimäärittely)
- Webcamin uudelleenalustus jos kuvanluku epäonnistuu
