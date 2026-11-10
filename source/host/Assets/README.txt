PantheraHT六轴机械臂：PantheraHT.urdf + meshes/。
模型来自HighTorque-Robotics/Panthera-HT_SDK官方固定版本，来源和哈希见source_manifest.json，许可见LICENSE。
已移除用户拆掉的原装爪子；质量惯量保留原始六轴模型参数。灵巧手/转接件的质量与安装变换尚未加入。
URDF的meshes路径相对于本目录；Pinocchio可将此目录加入package_dirs。
模型坐标尚未与电机编码器零位/方向标定。状态示例打印编码器SI值，不会将URDF限位用于软件clamp。
