# Tópicos do TIGO

Contrato entre as três. Mudou um tópico? Atualize este arquivo no mesmo Pull Request.

| Tópico | Tipo | Quem publica | Quem assina | Roda em |
|---|---|---|---|---|
| /cmd_vel | geometry_msgs/Twist | tigo_control | tigo_base | PC -> Pi |
| /odom | nav_msgs/Odometry | tigo_base | tigo_control, tigo_vision | Pi -> PC |
| /scan | sensor_msgs/LaserScan | tigo_sensors | tigo_vision | Pi -> PC |
| /camera/image_raw/compressed | sensor_msgs/CompressedImage | tigo_sensors | tigo_vision | Pi -> PC |

Os nomes acima são o ponto de partida. Ajustem conforme o hardware real.
