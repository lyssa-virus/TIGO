# Tópicos do TIGO

Contrato entre as três. Mudou um tópico? Atualize este arquivo no mesmo Pull Request.

| Tópico | Tipo | Quem publica | Quem assina | Roda em |
|---|---|---|---|---|
| /cmd_vel | geometry_msgs/Twist | tigo_control, tigo_telemetry | tigo_base | PC -> Pi |
| /odom | nav_msgs/Odometry | tigo_base | tigo_control, tigo_vision, tigo_telemetry | Pi -> PC |
| /tigo/pid_debug | std_msgs/Float64MultiArray | tigo_base | tigo_telemetry | Pi -> PC |
| /tigo/pid_gains | std_msgs/Float64MultiArray | tigo_telemetry | tigo_base | PC -> Pi |
| /scan | sensor_msgs/LaserScan | tigo_sensors | tigo_vision | Pi -> PC |
| /camera/image_raw/compressed | sensor_msgs/CompressedImage | tigo_sensors | tigo_vision | Pi -> PC |

Os nomes acima são o ponto de partida. Ajustem conforme o hardware real.
