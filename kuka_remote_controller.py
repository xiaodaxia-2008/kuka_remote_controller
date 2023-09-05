import re
import sys
import time
from collections import OrderedDict
from typing import List, Optional, Dict

from loguru import logger

from kuka_c3_bridge_client import C3BridgeClient

logger.remove(0)
logger.add(sys.stderr, level="INFO")


class KUKARemoteController:
    """KUKARemoteController is a API to control KUKA robot via socket.
    NOTE: unit of xyz position is millimeter by default;
    and unit of rpy is angle by default.
    """

    def __init__(self, ip: str = "172.31.1.147", port: int = 7000) -> None:
        """KUKARemoteController

        Args:
            ip (str, optional): KUKA controller IP. Defaults to "192.168.1.147".
            port (int, optional): KUKA controller port. Defaults to 7000.
        """
        self.c3client = C3BridgeClient(ip, port)
        if not self.c3client.can_connect:
            logger.error(f"failed to connect to KUKA controller ({ip=}, {port=})")
        self.pattern_reply_numbers = re.compile("\W(-?[\d\.]+)\W")

    def get_tool(self) -> Optional[List[float]]:
        """Get robot TOOL

        Returns:
            Optional[List[float]]: Robot TOOL, eg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], its a
            KUKA Frame, we convert it to xyzrpy
        """
        reply = self.c3client.read("$TOOL")
        if reply:
            return [
                float(q)
                for q in self.pattern_reply_numbers.findall(reply.decode("utf-8"))
            ]

    def set_tool(self, xyzrpy: List[float]) -> bool:
        """Set Robot TOOL

        Args:
            xyzrpy (List[float]): Robot TOOL , eg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        Returns:
            bool: True if success, False otherwise
        """
        vars = OrderedDict()
        vars["COM_FRAME"] = f"{{FRAME: {self.pose_to_str(xyzrpy)}}}"
        vars["COM_ACTION"] = "5"
        return self.__send_vars(vars)

    def get_pose(self) -> Optional[List[float]]:
        """Get robot tip POSE

        Returns:
            Optional[List[float]]: Robot POSE in xyzrpy, eg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        """
        reply = self.c3client.read("$POS_ACT_MES")
        if reply:
            numbers = self.pattern_reply_numbers.findall(reply.decode("utf-8"))
            xyzrpy = [float(num) for num in numbers[:6]]
            s = int(numbers[6])
            t = int(numbers[7])
            external_axes = [float(num) for num in numbers[8:]]
            logger.debug(f"{xyzrpy=}, {s=}, {t=}, {external_axes=}")
            return xyzrpy

    def get_joints(self) -> Optional[List[float]]:
        """Get robot joint position

        Returns:
            Optional[List[float]]: Robot joint position, first 6 is robot joints,
            then external axis joints
        """
        reply = self.c3client.read("$AXIS_ACT_MEAS")
        if reply:
            joints = [
                float(q)
                for q in self.pattern_reply_numbers.findall(reply.decode("utf-8"))
            ]
            return joints

    def movej(self, q: List[float], wait: bool = True) -> bool:
        """Move robot joint

        Args:
            q (List[float]): target joint positions
            wait (bool, optional): blocking or not. Defaults to True.

        Returns:
            bool: True if success
        """
        vars = OrderedDict()
        vars["COM_E6AXIS"] = f"{{E6AXIS: {self.joints_to_str(q)}}}"
        vars["COM_ACTION"] = "2"
        res = self.__send_vars(vars)
        if not res:
            return False
        if wait:
            self.__wait_motion()
        return bool(res)

    def movel(self, xyzrpy: List[float], wait: bool = True) -> bool:
        """Move robot tip to a target pose/frame

        Args:
            xyzrpy (List[float]): target pose
            wait (bool, optional): blocking or not. Defaults to True.

        Returns:
            bool: True if success
        """
        vars = OrderedDict()
        vars["COM_E6POS"] = f"{{E6POS: {self.pose_to_str(xyzrpy)}}}"
        vars["COM_ACTION"] = "3"
        res = self.__send_vars(vars)
        if not res:
            return False
        if wait:
            self.__wait_motion()
        return bool(res)

    def get_cartesian_speed(self) -> float:
        """get tip cartesian speed

        Returns:
            float: speed, unit is m/s
        """
        v = self.c3client.read("$VEL.CP")
        return float(v.decode())

    def get_joint_speed(self) -> float:
        """get joint speed, unit is deg/s.
        Note that each joint is set to a same joint speed by default.

        Returns:
            float: joint speed
        """
        v = self.c3client.read("$VEL_AXIS[1]")
        return float(v.decode())

    def set_cartesian_speed(self, meter_per_second: float) -> bool:
        """Set robot tip cartesian speed

        Args:
            meter_per_second (float): desired cartesian speed, m/s

        Returns:
            bool: True if success
        """
        vars = OrderedDict()
        vars["COM_VALUE1"] = f"{meter_per_second:.4f}"
        vars["COM_ACTION"] = "6"
        return self.__send_vars(vars)

    def set_joint_speed(self, deg_per_second: float) -> bool:
        """Set robot joints to the given speed

        Args:
            deg_per_second (float): desired speed, deg/s

        Returns:
            bool: True if success
        """
        vars = OrderedDict()
        vars["COM_VALUE2"] = f"{deg_per_second:.4f}"
        vars["COM_ACTION"] = "7"
        return self.__send_vars(vars)

    def set_digital_output(self, do_id: int, do_val: bool) -> bool:
        """Set robot digital output

        Args:
            do_id (int): digital output id
            do_val (bool): digital output value

        Returns:
            bool: True if success
        """
        vars = OrderedDict()
        vars["COM_VALUE1"] = f"{do_id}"
        vars["COM_VALUE2"] = f"{int(do_val)}"
        vars["COM_ACTION"] = "10"
        return self.__send_vars(vars)

    @staticmethod
    def pose_to_str(xyzrpy: List[float]) -> str:
        return ", ".join((f"{n} {v:.4f}" for n, v in zip("XYZABC", xyzrpy[:6])))

    @staticmethod
    def joints_to_str(q: List[float]) -> str:
        qstr = ""
        if len(q) >= 1:
            qstr += f"A1 {q[0]:.3f}"
        if len(q) >= 2:
            qstr += f", A2 {q[1]:.3f}"
        if len(q) >= 3:
            qstr += f", A3 {q[2]:.3f}"
        if len(q) >= 4:
            qstr += f", A4 {q[3]:.3f}"
        if len(q) >= 5:
            qstr += f", A5 {q[4]:.3f}"
        if len(q) >= 6:
            qstr += f", A6 {q[5]:.3f}"
        if len(q) >= 7:
            qstr += f", E1 {q[6]:.3f}"
        if len(q) >= 8:
            qstr += f", E2 {q[7]:.3f}"
        if len(q) >= 9:
            qstr += f", E3 {q[8]:.3f}"
        if len(q) >= 10:
            qstr += f", E4 {q[9]:.3f}"
        if len(q) >= 11:
            qstr += f", E5 {q[10]:.3f}"
        if len(q) >= 12:
            qstr += f", E6 {q[11]:.3f}"
        return qstr

    def __send_vars(self, vars: Dict[str, str]) -> bool:
        """Write variables to KUKA controller

        Args:
            vars (Dict[str, str]): variable names and values

        Returns:
            bool: True if success
        """
        res = None
        for k, v in vars.items():
            res = self.c3client.write(k, v)
            if res is None:
                logger.error(f"failed to write varialbe {k} to {v}")
                return False
        return res is not None

    def __wait_motion(self):
        """Blocking until robot motion finished"""
        logger.info("waiting task to be finished")
        while True:
            if self.c3client.read("COM_ACTION") == b"0":
                break
            time.sleep(0.1)


if __name__ == "__main__":
    from IPython import embed
    import numpy as np

    controller = KUKARemoteController(ip="192.168.1.147")
    p = controller.get_pose()
    q0 = controller.get_joints()
    q1 = q0[:]
    controller.set_cartesian_speed(0.5)
    controller.set_joint_speed(50)
    q1[0] -= 20
    res = controller.movej(q1)
    controller.set_tool([0.0, 0.0, 100.0, 0, 0, 0])
    res = controller.movel(p)
    q2 = controller.get_joints()
    error = np.array(q2) - np.array(q0)
    logger.info(f"{error=}")
    controller.set_digital_output(10, True)
    embed()
