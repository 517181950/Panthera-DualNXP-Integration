"""HD1910M外部机构行程标定。默认只读，运动必须加 --execute；--help 查看参数。"""
from tools.servo_travel_calibration import main

if __name__ == '__main__':
    raise SystemExit(main())
