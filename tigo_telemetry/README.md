# TIGO Telemetry (Aplicativo Desktop Ubuntu)

Aplicativo desktop nativo (**Python 3 + PyQt5**) para monitoramento em tempo real do robô mecanum **TIGO**, exibindo:
- Velocidade comandada (`/cmd_vel` - `geometry_msgs/Twist`) + indicador vetorial 2D + gráfico temporal
- Odometria (`/odom` - `nav_msgs/Odometry`) + mapa 2D interativo com rastro e botão limpar
- Controle PID por roda (`FL`, `FR`, `RL`, `RR`) com Setpoint vs. Medido, erro, saída PWM e ganhos `Kp`, `Ki`, `Kd`
- **Modo Simulação (Mock)** offline (20–100 Hz), botão **Pausar/Retomar** com inspeção por crosshair e **Exportar CSV**

## Como executar no seu computador

Não precisa instalar Node.js nem pacotes extras — usa apenas Python 3 e PyQt5 do Ubuntu:

```bash
cd /home/julia/controle_tigo/tigo_ws/tigo_telemetry
python3 main.py
```

## Modos de conexão
1. **Mock:** já inicia ativo para testar todos os gráficos e o mapa 2D com o robô desligado.
2. **ROS 2 (rclpy):** se você rodar `source /opt/ros/humble/setup.bash` antes de `python3 main.py`, o aplicativo assina diretamente os tópicos ROS 2 sem precisar de `rosbridge`.
3. **Rosbridge (WS):** conecta via WebSocket em `ws://IP:9090` (útil quando o ROS 2 roda em Docker ou outro computador).
