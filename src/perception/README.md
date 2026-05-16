# perception

Kasvojentunnistuspaketti. Tilaa kamerakuvaa, tunnistaa kasvot, analysoi ne ja julkaisee tulokset.

## FaceDetectionNode

Pääsolmu, joka suorittaa kasvojentunnistuksen.

**Tilaajat:**
- `image_topic` (`sensor_msgs/Image`) – raaka kamerakuva (oletus: `/i2e_webcam`)

**Julkaisijat:**
- `face_topic` (`face_tracker_msgs/Faces`) – tunnistettujen kasvojen lista (oletus: `/faces`)
- `face_image_topic` (`sensor_msgs/Image`) – annotoitu kuva (oletus: `face_detection`)

**Toiminta:**
1. Vastaanottaa kamerakuvan
2. `FaceAnalyzer` tunnistaa kasvot (5. välein korrelaatioseuranta, muuten joka kuva)
3. Laskee kasvojen embeddingsit ja ajaa klusteroinnin (`LinksCluster`)
4. Tunnistaa puhumisen huulten liikkeestä (`LipMovementDetector`)
5. Julkaisee tulokset

## FaceAnalyzer

Ydinluokka, joka prosessoi kuvan ja palauttaa listan tunnetuista kasvoista.
Tukee kasvojentunnistusta (DeepFace SFace), korrelaatioseurantaa (dlib) ja puhumisentunnistusta.

## LipMovementDetector

Keras-pohjainen RNN-malli, joka luokittelee puhuuko kasvojen omistaja. Perustuu huulten alueen landmark-pisteisiin (dlib shape predictor 68 landmarks).

## FaceRecognizer

DeepFace-kääre, joka tunnistaa kasvot kuvasta ja laskee niille embedding-vektorit.

## LinksCluster

Online-klusterointialgoritmi kasvojen tunnistamiseen. Yhdistää saman henkilön havainnot (face_id) käyttäen kosininimilariteettia.

## ROS2-parametrit

| Parametri | Tyyppi | Oletus | Kuvaus |
|-----------|--------|--------|--------|
| `image_topic` | string | `/i2e_webcam` | Kamerakuvan topic |
| `face_image_topic` | string | `face_detection` | Annotoidun kuvan topic |
| `face_topic` | string | `faces` | Kasvolistan topic |
| `lip_motion_model` | string | `1_32_False_True_0.25_lip_motion_net_model.h5` | Lip movement -mallin nimi |
| `shape_predictor` | string | `shape_predictor_68_face_landmarks.dat` | Dlib landmark-predictorin nimi |
| `face_recognizer_enabled` | bool | true | Käytetäänkö tunnistusta |
| `correlation_tracker` | bool | true | Käytetäänkö korrelaatioseurantaa |
| `cluster_similarity_threshold` | double | 0.3 | Klusterien samankaltaisuuskynnys |
| `subcluster_similarity_threshold` | double | 0.2 | Aliklusterien samankaltaisuuskynnys |
| `pair_similarity_maximum` | double | 1.0 | Parin maksimi samankaltaisuus |
| `face_recognition_model` | string | SFace | Tunnistusmalli |
| `face_detection_model` | string | yunet | Havaitsemismalli |

## Riippuvuudet

- `face_tracker_msgs` (Faces.msg)
- `core` (node_runner)
- `sensor_msgs/Image`
- deepface, dlib, tensorflow/keras, opencv-python
