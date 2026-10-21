# Tigo

Robô mecanum. Workspace ROS 2 Humble (Ubuntu 22.04).
Hardware: driver Moebius, Arduino Mega 2560, Raspberry Pi, lidar, câmera Nuwa HP60C.

## Onde cada pacote roda
- Na Pi: tigo_driver, tigo_sensores, tigo_bringup
- No PC: tigo_percepcao, tigo_navegacao, tigo_bringup
- firmware/: sketches do Arduino

## Fluxo de trabalho
1. Cada uma trabalha na própria branch (alyssa, julia ou roberta).
2. Nada de push direto na main: tudo entra por Pull Request.
3. Na Pi, só se faz git pull e build. Commits saem do PC.
