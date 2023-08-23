# KUKA Remote Controller

Control KUKA robot remotely via socket.

# Configuration
- 设置机器人网络IP，将电脑和机器人控制器内KLI网口连接
- 将[C3Bridge.exe](./application/C3Bridge.exe)复制到库卡的windows系统中，并设置开机启动。
- 复制[RoboDKsync562.src](./application/RoboDKsync562.src)复制到库卡控制器，并且自动运行。

详细步骤，[参考这里](./application/KUKARobotConfig.pdf)