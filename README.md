# **Laser Assembler Node**

Ce nœud ROS 2 (`laser_assembler`) souscrit à un topic `LaserScan`, transforme les scans en nuages de points et les accumule au fil du temps. Il fournit un service permettant d’assembler tous les nuages accumulés en un seul message `PointCloud2`.

---

## **Dépendances**

Assurez-vous d’avoir les dépendances suivantes installées :

```bash
pip install ros2_numpy open3d scipy
```

En plus, installez les paquets ROS 2 nécessaires :

```bash
sudo apt install ros-${ROS_DISTRO}-tf-transformations
```

---

## **Topics & Services**

### **Topics souscrits**
- `/perception/transformed_scan` (`sensor_msgs/LaserScan`) : Données brutes du laser scanner.

### **Services fournis**
- `assemble_cloud` (`encodeur/AssembleCloud`) : Assemble tous les nuages accumulés en un seul `PointCloud2`.

---

## **Utilisation**

### **1. Compiler le package**

Assurez-vous que votre espace de travail ROS 2 est bien configuré, puis compilez le package :

```bash
cd ~/ros2_ws
colcon build --packages-select <nom_du_package>
source install/setup.bash
```

### **2. Lancer le nœud**

Exécutez le nœud avec :

```bash
ros2 run laser_assembler laser_assembler
```

### **3. Appeler le service**

Pour assembler les nuages accumulés, appelez le service :

```bash
ros2 service call /assemble_cloud encodeur/srv/AssembleCloud
```

---

## **Configuration**

- Le nœud transforme les données `LaserScan` entrantes dans le repère `r_robot`. Vous pouvez modifier cela en changeant la variable `self.global_frame` dans la classe `LaserAssemblerNode`.
- Il utilise `tf2_ros` pour obtenir les transformations et nécessite une diffusion correcte des TF depuis votre configuration robotique.
- Le nœud utilise Open3D pour la gestion des nuages de points.

---

## **Remarques**

- Vérifiez que votre arborescence TF est bien configurée et que la transformation du laser vers `r_robot` est disponible.
- Si `ros2_numpy` n’est pas installé, vous pouvez l’installer avec :

```bash
pip install ros2_numpy
```

---

## **Dépannage**

- Si le nœud ne reçoit aucun message `LaserScan`, vérifiez que le topic est bien publié :

```bash
ros2 topic list
```

- En cas d’erreur de transformation TF, assurez-vous que la transformation existe :

```bash
ros2 run tf2_ros tf2_echo <frame_cible> <frame_source>
```

- Consultez les logs pour voir les erreurs :

```bash
ros2 run <nom_du_package> laser_assembler --ros-args --log-level debug
```

---
