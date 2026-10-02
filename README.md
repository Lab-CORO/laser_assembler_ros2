# **laser_assembler**

Assemble une succession de balayages `LaserScan` 2D en un nuage de points 3D (`PointCloud2`) exprimé dans un repère fixe (`r_robot` par défaut).

Le package fournit :

- **`ScanAssembler`** (`laser_assembler/scan_assembler.py`) : une classe Python à utiliser directement dans votre node. **C'est ce que le laboratoire utilise.** Aucun service ni executor multi-thread requis.
- **`laser_assembler`** (node, facultatif) : enveloppe ROS autour de `ScanAssembler` (topic + service `assemble_cloud`).

---

## **Dépendances**

```bash
pip install numpy scipy
sudo apt install ros-${ROS_DISTRO}-tf2-ros-py
```

---

## **Utilisation de la classe ScanAssembler**

Pour chaque scan, la classe lit dans TF la transformation `fixed_frame ← frame du scan` (au temps du scan), transforme les points, puis les stocke.

```python
import tf2_ros
from laser_assembler.scan_assembler import ScanAssembler

# dans le constructeur de votre node
# Le listener a son propre node et son propre thread : le buffer reste a jour meme si un
# callback long (action) occupe votre node. Ne pas lui passer `self` (le meme node se
# retrouverait dans deux executors et TF ne serait plus mis a jour sous Humble).
self.tf_buffer = tf2_ros.Buffer()
self.tf_node = rclpy.create_node('mon_node_tf')
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
self.assembler = ScanAssembler(self.tf_buffer, fixed_frame='r_robot')

# pour chaque nouveau balayage (le frame_id du scan doit etre celui diffuse dans TF)
self.assembler.add_scan(scan_msg)          # False si TF indisponible

# a la fin du balayage
cloud = self.assembler.to_pointcloud2()    # sensor_msgs/PointCloud2 dans r_robot
self.assembler.clear()                     # pret pour un nouveau balayage
```

| Membre | Rôle |
|---|---|
| `ScanAssembler(tf_buffer, fixed_frame='r_robot', max_scans=None)` | `max_scans` borne le nombre de scans gardés (tampon circulaire) |
| `add_scan(scan)` | Ajoute un `LaserScan` ; ignore les points `inf`/hors portée |
| `to_pointcloud2(stamp=None)` | Fusionne tous les scans en un `PointCloud2` |
| `clear()` | Vide le tampon |
| `n_scans`, `n_points` | Taille du tampon |

---

## **Node facultatif**

Souscrit à `/perception/transformed_scan` (`sensor_msgs/LaserScan`) et offre le service `assemble_cloud` (`encodeur/AssembleCloud`), qui retourne le nuage accumulé puis vide le tampon.

```bash
ros2 run laser_assembler laser_assembler
ros2 service call /assemble_cloud encodeur/srv/AssembleCloud
```

---

## **Tests**

```bash
cd laser_assembler_ros2
python3 -m pytest test/test_scan_assembler.py
```

---

## **Dépannage**

- Si `add_scan` retourne `False`, la transformation TF `r_robot → <frame du scan>` n'existe pas encore :

```bash
ros2 run tf2_ros tf2_echo r_robot laser
```

- Si le nuage semble retourné de 180°, vérifiez la transformation diffusée (matrice `^R T_L` du Jalon 1), pas l'assembleur : celui-ci applique exactement la TF lue.
