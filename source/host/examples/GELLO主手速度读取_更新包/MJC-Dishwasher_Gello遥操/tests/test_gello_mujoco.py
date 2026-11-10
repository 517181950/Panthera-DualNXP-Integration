from __future__ import annotations

from dataclasses import replace
import time
import unittest

import mujoco

from dishwasher_sim import DishwasherSim
from gello import (
    AxisConfig,
    GelloConfig,
    GripperConfig,
    GroupSnapshot,
    ReaderSnapshot,
    map_group_to_targets,
)
from run_gello import apply_snapshot


def make_group(
    group: int,
    *,
    arm_radians: tuple[float | None, ...] = (0.0,) * 6,
    gripper_raw: int | None = 2048,
    gripper_unwrapped: int | None = 2048,
    gripper_normalized: float | None = 0.5,
    data_valid: bool = True,
    zeroed: bool = True,
) -> GroupSnapshot:
    now = time.monotonic()
    servo_ids = tuple(range(1 + group * 10, 8 + group * 10))
    return GroupSnapshot(
        group=group,
        servo_ids=servo_ids,
        raw_positions=(1000,) * 6 + (gripper_raw,),
        unwrapped_ticks=(1000,) * 6 + (gripper_unwrapped,),
        arm_radians=arm_radians,
        gripper_radians=0.0,
        gripper_normalized=(
            None if gripper_raw is None else gripper_normalized
        ),
        gripper_calib_min_raw=0,
        gripper_calib_max_raw=4095,
        gripper_calib_active=False,
        gripper_calib_done=True,
        zeroed=zeroed,
        data_valid=data_valid,
        captured_at=now,
    )


class GelloMappingTests(unittest.TestCase):
    def test_zero_pose_maps_to_mujoco_neutral_pose(self) -> None:
        targets = map_group_to_targets(
            make_group(0),
            GelloConfig().left,
            DishwasherSim.INITIAL_ARM_ANGLES,
        )

        self.assertEqual(targets.joints, DishwasherSim.INITIAL_ARM_ANGLES)
        self.assertAlmostEqual(
            targets.gripper,
            GelloConfig().left.gripper.apply_fraction(0.5),
        )

    def test_joint_and_gripper_mapping_parameters_are_applied(self) -> None:
        config = GelloConfig().left
        config.axes[0] = AxisConfig(sign=-1, scale=2.0, offset_rad=0.1)
        config.gripper = GripperConfig(
            min_raw=1000,
            max_raw=3000,
            inverted=True,
            output_min=0.002,
            output_max=0.037,
        )

        targets = map_group_to_targets(
            make_group(
                0,
                arm_radians=(0.25, 0.0, 0.0, 0.0, 0.0, 0.0),
                gripper_normalized=0.4,
            ),
            config,
            DishwasherSim.INITIAL_ARM_ANGLES,
        )

        self.assertAlmostEqual(targets.joints[0], -0.4)
        self.assertAlmostEqual(targets.joints[1], 0.0)
        self.assertAlmostEqual(targets.gripper, 0.023)

    def test_gripper_mapping_uses_continuous_normalized_position(self) -> None:
        config = GelloConfig().left
        config.gripper = GripperConfig(
            min_raw=4000,
            max_raw=4200,
            output_min=0.0,
            output_max=0.6,
        )

        targets = map_group_to_targets(
            make_group(
                0,
                gripper_raw=104,
                gripper_unwrapped=4200,
                gripper_normalized=1.0,
            ),
            config,
            DishwasherSim.INITIAL_ARM_ANGLES,
        )

        self.assertAlmostEqual(targets.gripper, 0.6)

    def test_incomplete_or_unzeroed_group_is_rejected(self) -> None:
        for group in (
            make_group(0, data_valid=False),
            make_group(0, zeroed=False),
            make_group(0, gripper_raw=None),
        ):
            with self.subTest(group=group):
                with self.assertRaises(ValueError):
                    map_group_to_targets(
                        group,
                        GelloConfig().left,
                        DishwasherSim.INITIAL_ARM_ANGLES,
                    )


class GelloMujocoIntegrationTests(unittest.TestCase):
    def test_initial_pose_has_no_contact_between_arms(self) -> None:
        sim = DishwasherSim()
        inter_arm_contacts = []
        for contact in sim.data.contact:
            body_ids = (
                int(sim.model.geom_bodyid[contact.geom1]),
                int(sim.model.geom_bodyid[contact.geom2]),
            )
            body_names = tuple(
                mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                or "world"
                for body_id in body_ids
            )
            if (
                body_names[0].startswith("left/")
                and body_names[1].startswith("right/")
            ) or (
                body_names[0].startswith("right/")
                and body_names[1].startswith("left/")
            ):
                inter_arm_contacts.append(body_names)

        self.assertEqual(inter_arm_contacts, [])

    def test_complete_dual_arm_snapshot_drives_mujoco(self) -> None:
        left = make_group(
            0, arm_radians=(0.1, 0.05, -0.1, 0.02, -0.03, 0.04)
        )
        right = make_group(
            1, arm_radians=(-0.1, -0.05, 0.1, -0.02, 0.03, -0.04)
        )
        snapshot = ReaderSnapshot(
            groups=(left, right),
            captured_at=time.monotonic(),
            sequence=3,
            scan_duration_s=0.01,
        )
        sim = DishwasherSim()

        applied = apply_snapshot(sim, snapshot, GelloConfig())
        sim.step(10)

        self.assertEqual(applied, ("left", "right"))
        self.assertAlmostEqual(sim.data.actuator("left/joint1").ctrl[0], 0.1)
        self.assertAlmostEqual(sim.data.actuator("right/joint1").ctrl[0], -0.1)

    def test_swap_arms_exchanges_group_targets_but_keeps_group_config(self) -> None:
        group_0 = make_group(0, arm_radians=(0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
        group_1 = make_group(1, arm_radians=(-0.1, 0.0, 0.0, 0.0, 0.0, 0.0))
        snapshot = ReaderSnapshot(
            groups=(group_0, group_1),
            captured_at=time.monotonic(),
            sequence=4,
            scan_duration_s=0.01,
        )
        config = GelloConfig(swap_arms=True)
        config.left.axes[0] = AxisConfig(scale=2.0)
        config.right.axes[0] = AxisConfig(scale=3.0)
        sim = DishwasherSim()

        applied = apply_snapshot(sim, snapshot, config)

        self.assertEqual(applied, ("right", "left"))
        self.assertAlmostEqual(sim.data.actuator("right/joint1").ctrl[0], 0.2)
        self.assertAlmostEqual(sim.data.actuator("left/joint1").ctrl[0], -0.3)

    def test_incomplete_arm_does_not_block_complete_other_arm(self) -> None:
        left = replace(make_group(0), data_valid=False)
        right = make_group(1, arm_radians=(0.1,) * 6)
        snapshot = ReaderSnapshot(
            groups=(left, right),
            captured_at=time.monotonic(),
            sequence=4,
            scan_duration_s=0.01,
        )
        sim = DishwasherSim()

        applied = apply_snapshot(sim, snapshot, GelloConfig())

        self.assertEqual(applied, ("right",))
        self.assertAlmostEqual(sim.data.actuator("left/joint1").ctrl[0], 0.0)
        self.assertAlmostEqual(sim.data.actuator("right/joint1").ctrl[0], 0.1)


if __name__ == "__main__":
    unittest.main()
