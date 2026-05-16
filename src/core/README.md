# core

Yleiskäyttöinen kirjastopaketti, joka tarjoaa pohjaluokat sensorisolmuille.

## SensorBase

Abstrakti kantaluokka yksittäisille sensoreille. Kaikkien sensoritoteutusten tulee periä tämä luokka ja toteuttaa `read()`-metodi.

```python
class OmaSensor(SensorBase):
    def read(self) -> None:
        # Lue data ja julkaise se
        pass
```

## SensorNodeBase

ROS2-solmu, joka hallinnoi useita `SensorBase`-sensoreita. Sensorit lisätään `add_sensor()`-metodilla ja niiden `read()`-metodia kutsutaan `read_sensors()`-kutsulla.

```python
class OmaNode(SensorNodeBase):
    def __init__(self):
        super().__init__("node_name")
        self.add_sensor("sensor_name", OmaSensor)
        self.create_timer(0.1, self.read_sensors)
```

## util

`run_node(node_class)` – Apufunktio joka alustaa rclpy:n, käynnistää solmun ja hoitaa siistin sammutuksen.

## Riippuvuudet

- rclpy
- Ei muita ROS2-paketteja

## ROS2-parametrit

Ei omia parametreja – toimii kirjastopakettina muille solmuille.
