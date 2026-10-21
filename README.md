# Tigo

Robô mecanum. Workspace ROS 2 Humble (Ubuntu 22.04).
Hardware: driver Möbius, Arduino Mega 2560, Raspberry Pi, lidar, câmera Nuwa HP60C.

## Pacotes e onde rodam
- Na Pi: tigo_base, tigo_sensors, tigo_bringup
- No PC: tigo_control, tigo_vision, tigo_bringup
- firmware/: sketches do Arduino
- docs/TOPICOS.md: contrato de tópicos entre os pacotes

## Fluxo de trabalho
1. Cada uma trabalha na própria branch (alyssa, julia ou roberta), em qualquer pacote.
2. Nada de push direto na main: tudo entra por Pull Request.
3. Na Pi, só se faz git pull e build. Commits saem do PC.
4. Antes de mexer num pacote que outra pessoa também está editando, avisar no grupo.
5. Pull Requests pequenos, e trazer a main para a sua branch com frequência.
