"""Checks that the model is self-consistent: each equation solved forwards and backwards agrees, and
the model still reproduces the Crazyflie 2.1 Brushless.

Run from the project folder:  python -m unittest discover tests
"""

import math
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import reference
from drone_sizing.airframe import Airframe
from drone_sizing.battery import LIPO, BatteryKind
from drone_sizing.build import Build, Uncertainty
from drone_sizing.catalog import load_batteries, load_motor_scaling, load_motors, load_propellers
from drone_sizing.closure import MassDidNotConverge, close_mass
from drone_sizing.constants import GRAVITY_M_PER_S2, METERS_PER_MILLIMETER
from drone_sizing.inputs import DesignChoices, Requirements, ScalingLaws, Scenario, Technology
from drone_sizing.motor import REFERENCE_SPEED_RAD_S, MotorScaling, StatorSize, nearest_stator_sizes
from drone_sizing.numerics import PowerLaw, minimize, solve_increasing
from drone_sizing.propeller import Propeller, PropellerLaw, PropellerScaling
from drone_sizing.search import search
from drone_sizing.typical import PropSizes, TypicalFrame, TypicalParts, TypicalPropeller, best_drone, drones_by_prop_size, lightest

SPEED_EXPONENT = 0.5
MOTOR = reference.MOTOR
PROPELLER = reference.propeller(SPEED_EXPONENT)
BATTERY = reference.BATTERY

# A made-up prop rule, just to exercise the code.
PROP_SCALING = PropellerScaling(
    thrust_coefficient=PropellerLaw(coefficient=0.1, pitch_ratio_exponent=0.8, diameter_exponent=0.1),
    power_coefficient=PropellerLaw(coefficient=0.05, pitch_ratio_exponent=1.5, diameter_exponent=-0.1),
    blade_factors=((3, 1.2, 1.3),),
)

TECH = Technology(
    usable_battery_fraction=0.8,
    lead_resistance_ohm=0.015,
    no_load_current_speed_exponent=SPEED_EXPONENT,
)
REQUIREMENTS = Requirements(
    payload_mass_kg=0.0,
    flight_time_s=7 * 60,
    thrust_to_weight=2.0,
    average_power_factor=1.2,
    mass_margin_fraction=0.10,
)

# A kind of battery like the reference pack, for sizing.
REFERENCE_PACK_KIND = BatteryKind("like the reference pack", 1, BATTERY.energy_wh / BATTERY.mass_kg, LIPO)
UNCERTAINTY = Uncertainty(0.1, 0.1, 0.3, 0.1, 0.5, 0.25, 0.35, 0.3, 1.0)


class NumericsTest(unittest.TestCase):
    def test_solve_increasing_finds_the_root(self):
        self.assertAlmostEqual(solve_increasing(lambda x: x**3, 8.0, 0.0, 10.0), 2.0, places=9)

    def test_minimize_finds_the_dip(self):
        self.assertAlmostEqual(minimize(lambda x: (x - 1.3) ** 2, 0.0, 5.0), 1.3, places=6)

    def test_power_law_fit_recovers_the_law(self):
        law = PowerLaw(coefficient=2.5, exponent=0.83)
        xs = [0.001, 0.003, 0.01, 0.05]
        fitted = PowerLaw.fit(xs, [law(x) for x in xs])
        self.assertAlmostEqual(fitted.exponent, 0.83, places=9)
        self.assertAlmostEqual(fitted.coefficient, 2.5, places=9)


class PropellerTest(unittest.TestCase):
    def test_coefficients_from_a_test_point_reproduce_it(self):
        speed_rad_s = 3000.0
        rebuilt = Propeller.from_test_point(
            name="rebuilt",
            diameter_m=PROPELLER.diameter_m,
            mass_kg=PROPELLER.mass_kg,
            thrust_n=PROPELLER.thrust_at_speed_n(speed_rad_s),
            speed_rad_s=speed_rad_s,
            shaft_power_w=PROPELLER.torque_at_speed_nm(speed_rad_s) * speed_rad_s,
        )
        self.assertAlmostEqual(rebuilt.thrust_coefficient, PROPELLER.thrust_coefficient)
        self.assertAlmostEqual(rebuilt.power_coefficient, PROPELLER.power_coefficient)

    def test_speed_for_thrust_inverts_thrust_at_speed(self):
        speed_rad_s = PROPELLER.speed_for_thrust_rad_s(0.1)
        self.assertAlmostEqual(PROPELLER.thrust_at_speed_n(speed_rad_s), 0.1)


class MotorTest(unittest.TestCase):
    def test_no_load_point_reproduces_the_datasheet(self):
        # Spinning freely at its test speed, the motor draws I_0 at the test voltage.
        point = MOTOR.operating_point(0.0, MOTOR.no_load_test_speed_rad_s, SPEED_EXPONENT)
        self.assertAlmostEqual(point.current_a, MOTOR.no_load_current_a)
        self.assertAlmostEqual(point.voltage_v, MOTOR.no_load_test_voltage_v)

    def test_full_throttle_uses_exactly_the_voltage_given(self):
        point = MOTOR.full_throttle(3.7, PROPELLER, SPEED_EXPONENT)
        self.assertAlmostEqual(point.voltage_v, 3.7, places=9)
        self.assertAlmostEqual(point.torque_nm, PROPELLER.torque_at_speed_nm(point.speed_rad_s))

    def test_measured_point_inverts_operating_point(self):
        point = MOTOR.operating_point(0.001, 2500.0, SPEED_EXPONENT)
        measured = MOTOR.measured_point(point.voltage_v, point.current_a, SPEED_EXPONENT)
        self.assertAlmostEqual(measured.speed_rad_s, 2500.0)
        self.assertAlmostEqual(measured.torque_nm, 0.001)

    def test_rewinding_changes_current_but_not_efficiency(self):
        # Half the turns: twice the Kv, a quarter of the resistance, twice the no-load current at
        # half the test voltage (same test speed). Same copper and iron, so same K_m and same drag.
        rewound = replace(
            MOTOR,
            kv_rpm_per_v=2 * MOTOR.kv_rpm_per_v,
            resistance_ohm=MOTOR.resistance_ohm / 4,
            no_load_current_a=2 * MOTOR.no_load_current_a,
            no_load_test_voltage_v=MOTOR.no_load_test_voltage_v / 2,
        )
        self.assertAlmostEqual(rewound.motor_constant_nm_per_sqrt_w, MOTOR.motor_constant_nm_per_sqrt_w)

        original_point = MOTOR.operating_point(0.0005, 1800.0, SPEED_EXPONENT)
        rewound_point = rewound.operating_point(0.0005, 1800.0, SPEED_EXPONENT)
        self.assertAlmostEqual(rewound_point.efficiency, original_point.efficiency)
        self.assertAlmostEqual(rewound_point.current_a, 2 * original_point.current_a)


class StatorSizeTest(unittest.TestCase):
    def test_code_is_written_the_way_listings_write_it(self):
        self.assertEqual(StatorSize(12, 3, 0.0045).code, "1203")
        self.assertEqual(StatorSize(12, 2.5, 0.0039).code, "1202.5")
        self.assertEqual(StatorSize(8, 2.8, 0.0024).code, "0802.8")
        self.assertEqual(StatorSize.from_code("1202.5", 0.0039), StatorSize(12, 2.5, 0.0039))

    def test_nearest_sizes_are_the_closest_in_mass_closest_first(self):
        sizes = [StatorSize.from_code(code, mass_kg) for code, mass_kg in (("0802", 0.0019), ("1103", 0.0040), ("1203", 0.0045), ("1404", 0.0104))]
        self.assertEqual([size.code for size in nearest_stator_sizes(sizes, 0.0042, 2)], ["1103", "1203"])


class MotorScalingTest(unittest.TestCase):
    def setUp(self):
        self.scaling = MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT)

    def test_scaling_from_one_motor_passes_through_it(self):
        self.assertAlmostEqual(self.scaling.motor_constant(MOTOR.mass_kg), MOTOR.motor_constant_nm_per_sqrt_w)
        self.assertAlmostEqual(
            self.scaling.drag_torque(MOTOR.mass_kg), MOTOR.drag_torque_nm(REFERENCE_SPEED_RAD_S, SPEED_EXPONENT)
        )
        self.assertAlmostEqual(
            self.scaling.max_copper_loss(MOTOR.mass_kg), MOTOR.max_current_a**2 * MOTOR.resistance_ohm
        )

    def test_fit_across_motors_recovers_the_scaling(self):
        motors = [self.scaling.motor(mass_kg, 0.001, 3000.0, 3.5, SPEED_EXPONENT) for mass_kg in (0.002, 0.004, 0.008)]
        fitted = MotorScaling.fit(motors, SPEED_EXPONENT)
        self.assertAlmostEqual(fitted.motor_constant.exponent, 5 / 6, places=9)
        self.assertAlmostEqual(fitted.drag_torque.exponent, 1.0, places=9)
        self.assertAlmostEqual(fitted.max_copper_loss.exponent, 2 / 3, places=9)

    def test_sizing_for_the_reference_motors_own_peak_gives_back_that_motor(self):
        # A motor of the 08028's mass, wound to reach the 08028's own full-throttle point, is the 08028.
        peak = MOTOR.full_throttle(reference.PEAK_VOLTAGE_V, PROPELLER, SPEED_EXPONENT)
        sized = self.scaling.motor(
            MOTOR.mass_kg, peak.torque_nm, peak.speed_rad_s, reference.PEAK_VOLTAGE_V, SPEED_EXPONENT
        )
        self.assertAlmostEqual(sized.kv_rpm_per_v, MOTOR.kv_rpm_per_v, places=6)
        self.assertAlmostEqual(sized.resistance_ohm, MOTOR.resistance_ohm)
        self.assertAlmostEqual(sized.max_current_a, MOTOR.max_current_a)

    def test_sized_motor_just_reaches_the_max_point(self):
        max_speed_rad_s = PROPELLER.speed_for_thrust_rad_s(0.2)
        max_torque_nm = PROPELLER.torque_at_speed_nm(max_speed_rad_s)
        sized = self.scaling.motor(0.003, max_torque_nm, max_speed_rad_s, 3.5, SPEED_EXPONENT)
        point = sized.full_throttle(3.5, PROPELLER, SPEED_EXPONENT)
        self.assertAlmostEqual(point.speed_rad_s / max_speed_rad_s, 1.0, places=9)

    def test_kv_window_edges_are_the_thrust_and_current_limits(self):
        max_speed_rad_s = PROPELLER.speed_for_thrust_rad_s(0.2)
        max_torque_nm = PROPELLER.torque_at_speed_nm(max_speed_rad_s)
        mass_kg, max_current_a = 0.003, 3.0
        low_kv, high_kv = self.scaling.kv_window(
            mass_kg, max_current_a, PROPELLER, max_torque_nm, max_speed_rad_s, 3.5, 4.0, SPEED_EXPONENT
        )

        # At the lowest Kv, full throttle on a tired pack just reaches the max point.
        slowest = self.scaling.motor_with_kv("low", low_kv, mass_kg)
        self.assertAlmostEqual(
            slowest.full_throttle(3.5, PROPELLER, SPEED_EXPONENT).speed_rad_s / max_speed_rad_s, 1.0, places=6
        )
        # At the highest Kv, full throttle on a fresh pack just reaches the rated current.
        fastest = self.scaling.motor_with_kv("high", high_kv, mass_kg)
        self.assertAlmostEqual(fastest.full_throttle(4.0, PROPELLER, SPEED_EXPONENT).current_a, max_current_a, places=6)


class ClosureTest(unittest.TestCase):
    def setUp(self):
        self.choices = DesignChoices(airframe=reference.AIRFRAME, propeller=PROPELLER)
        self.scaling = ScalingLaws(battery=REFERENCE_PACK_KIND, motor=MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT))

    def test_converged_design_weighs_what_it_was_designed_for(self):
        design = close_mass(REQUIREMENTS, self.choices, TECH, self.scaling)
        self.assertLess(abs(design.mass_mismatch_kg), 1e-5)

    def test_converged_design_meets_its_requirements_when_judged_as_a_build(self):
        # The sized parts, evaluated exactly as catalog parts are, fly the required time and reach
        # the required thrust-to-weight on the sagging tired pack, with the motor within its rating.
        design = close_mass(REQUIREMENTS, self.choices, TECH, self.scaling)
        result = design.result
        self.assertAlmostEqual(result.flight_time_s / REQUIREMENTS.flight_time_s, 1.0, places=3)
        self.assertAlmostEqual(result.thrust_to_weight / REQUIREMENTS.thrust_to_weight, 1.0, places=3)
        self.assertLessEqual(result.fresh_full_throttle.current_a, design.motor.max_current_a * (1 + 1e-6))
        self.assertEqual(result.mass_breakdown_kg, design.mass_breakdown_kg)

    def test_impossible_flight_time_is_reported_infeasible(self):
        with self.assertRaises(MassDidNotConverge):
            close_mass(replace(REQUIREMENTS, flight_time_s=60 * 60), self.choices, TECH, self.scaling)


class TypicalPartsTest(unittest.TestCase):
    """Sizing over prop sizes, with typical parts that all behave like the reference drone's."""

    def setUp(self):
        same_as_reference = PropellerScaling(
            thrust_coefficient=PropellerLaw(PROPELLER.thrust_coefficient, 0.0, 0.0),
            power_coefficient=PropellerLaw(PROPELLER.power_coefficient, 0.0, 0.0),
            blade_factors=(),
        )
        self.typical = TypicalParts(
            propeller=TypicalPropeller(same_as_reference, 2, 0.6, 1.0, PROPELLER.diameter_m, PROPELLER.mass_kg),
            frame=TypicalFrame(rotor_count=4, mass_kg_per_m=0.06, prop_clearance_m=0.008),
            motor=MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT),
        )
        self.scenario = Scenario("test", REQUIREMENTS, REFERENCE_PACK_KIND, reference.AIRFRAME.components)
        self.sizes = PropSizes(smallest_mm=40, largest_mm=80, coarse_step_mm=20, fine_step_mm=10)

    def test_a_typical_frame_just_fits_its_prop(self):
        propeller = self.typical.propeller.propeller(0.055)
        airframe = self.typical.frame.airframe(propeller, self.scenario.components)
        self.assertTrue(airframe.fits_prop(propeller.diameter_m))
        self.assertFalse(airframe.fits_prop(propeller.diameter_m * 1.01))

    def test_best_drone_is_the_lightest_over_the_sizes_tried(self):
        drones = drones_by_prop_size(self.scenario, self.typical, TECH, self.sizes)
        found = [drone for drone in drones.values() if drone is not None]
        self.assertTrue(found)
        best = lightest(drones)
        self.assertIs(best, min(found, key=lambda drone: drone.built_mass_kg))
        self.assertEqual(best_drone(self.scenario, self.typical, TECH, self.sizes).choices.propeller, best.choices.propeller)
        # Fine steps were added around the lightest coarse size.
        self.assertGreater(len(drones), len(self.sizes.coarse_mm))

    def test_an_impossible_scenario_gives_no_drone(self):
        hopeless = replace(self.scenario, requirements=replace(REQUIREMENTS, flight_time_s=60 * 60))
        self.assertIsNone(best_drone(hopeless, self.typical, TECH, self.sizes))


class BuildTest(unittest.TestCase):
    def setUp(self):
        self.build = Build(reference.AIRFRAME, MOTOR, PROPELLER, BATTERY)

    def test_reference_build_adds_up_to_the_published_takeoff_mass(self):
        result = self.build.evaluate(replace(REQUIREMENTS, mass_margin_fraction=0.0), TECH)
        self.assertAlmostEqual(result.total_mass_kg, reference.TAKEOFF_MASS_KG)

    def test_full_throttle_voltage_is_the_resting_voltage_minus_the_sag(self):
        result = self.build.evaluate(REQUIREMENTS, TECH)
        resistance_ohm = BATTERY.resistance_ohm(TECH.lead_resistance_ohm)
        for point, resting_voltage_v in (
            (result.tired_full_throttle, BATTERY.tired_voltage_v),
            (result.fresh_full_throttle, BATTERY.fresh_voltage_v),
        ):
            battery_current_a = self.build.full_throttle_battery_current_a(point, resting_voltage_v)
            self.assertAlmostEqual(point.voltage_v, resting_voltage_v - battery_current_a * resistance_ohm, places=6)
            self.assertLess(point.voltage_v, resting_voltage_v)

    def test_closed_form_sag_agrees_with_the_full_throttle_solve(self):
        # The sizing loop works the sagged voltage out in closed form from the motors' power; the
        # build finds it by searching over speed. At the same point they must agree.
        point = self.build.full_throttle_on_pack(BATTERY.tired_voltage_v, TECH)
        airframe = self.build.airframe
        motor_power_w = airframe.rotor_count * point.input_power_w / airframe.board.esc_efficiency
        other_current_a = airframe.electronics_power_w / BATTERY.tired_voltage_v
        voltage_v = BATTERY.voltage_under_load_v(BATTERY.tired_voltage_v, TECH.lead_resistance_ohm, motor_power_w, other_current_a)
        self.assertAlmostEqual(voltage_v, point.voltage_v, places=9)

    def test_a_pack_with_no_resistance_reproduces_bitcrazes_full_throttle_point(self):
        # The prop's coefficients came from Bitcraze's 4.0 V, 1.8 A point. A pack that rests at
        # 4.0 V and doesn't sag puts the motor back on that point.
        cells_at_peak_voltage = replace(LIPO, full_cell_voltage_v=reference.PEAK_VOLTAGE_V)
        stiff_pack = replace(BATTERY, internal_resistance_ohm=0.0, chemistry=cells_at_peak_voltage)
        result = replace(self.build, battery=stiff_pack).evaluate(REQUIREMENTS, TECH)
        self.assertAlmostEqual(result.fresh_full_throttle.current_a, reference.PEAK_CURRENT_A, places=6)

    def test_a_bigger_pack_sags_less(self):
        small = replace(self.build, battery=replace(BATTERY, capacity_ah=0.3)).evaluate(REQUIREMENTS, TECH)
        big = replace(self.build, battery=replace(BATTERY, capacity_ah=1.1)).evaluate(REQUIREMENTS, TECH)
        self.assertLess(small.tired_full_throttle.voltage_v, big.tired_full_throttle.voltage_v)

    def test_power_budget_adds_up(self):
        result = self.build.evaluate(REQUIREMENTS, TECH)
        power = result.power
        rotor_count = self.build.airframe.rotor_count
        self.assertAlmostEqual(power.hover_w, rotor_count * result.hover.input_power_w / self.build.airframe.board.esc_efficiency)
        self.assertAlmostEqual(
            power.total_w,
            REQUIREMENTS.average_power_factor * power.hover_w + self.build.airframe.electronics_power_w,
        )
        self.assertAlmostEqual(result.flight_time_s, result.usable_energy_j / power.total_w)
        self.assertGreater(min(power.lifting_w, power.propeller_loss_w, power.winding_loss_w, power.motor_drag_loss_w, power.esc_loss_w), 0.0)

    def test_removing_the_margins_only_makes_the_numbers_better(self):
        designed = self.build.evaluate(REQUIREMENTS, TECH)
        best = designed.best_estimate
        self.assertGreater(best.flight_time_s, designed.flight_time_s)
        self.assertLess(best.total_mass_kg, designed.total_mass_kg)
        self.assertGreater(best.fresh_thrust_to_weight, designed.thrust_to_weight)

    def test_model_reproduces_the_crazyflie_hover(self):
        # Bitcraze: over 5 g/W in hover, and about 10 minutes of flight. 10 minutes should use a
        # plausible share of the pack: more than 70%, less than all of it.
        hover_only = replace(REQUIREMENTS, average_power_factor=1.0, mass_margin_fraction=0.0)
        result = self.build.evaluate(hover_only, TECH)
        self.assertGreater(result.total_mass_kg * 1000 / result.average_battery_power_w, 5.0)
        pack_fraction = result.average_battery_power_w * reference.FLIGHT_TIME_S / BATTERY.energy_j
        self.assertTrue(0.7 < pack_fraction < 1.0, pack_fraction)

    def test_incompatible_parts_are_caught(self):
        big_prop = replace(PROPELLER, diameter_m=90 * METERS_PER_MILLIMETER)
        wrong_bore = replace(PROPELLER, bore_diameter_m=1.5 * METERS_PER_MILLIMETER)
        two_cell = replace(BATTERY, cell_count=2)
        self.assertEqual(self.build.incompatibilities(), [])
        self.assertEqual(len(replace(self.build, propeller=big_prop).incompatibilities()), 1)
        self.assertEqual(len(replace(self.build, propeller=wrong_bore).incompatibilities()), 1)
        self.assertEqual(len(replace(self.build, battery=two_cell).incompatibilities()), 1)

    def test_motor_rated_for_fewer_cells_is_incompatible(self):
        build = Build(reference.AIRFRAME, replace(MOTOR, max_cell_count=1), PROPELLER, replace(BATTERY, cell_count=2))
        two_cell_board = replace(reference.AIRFRAME.board, supported_cell_counts=(1, 2))
        airframe = replace(reference.AIRFRAME, components=(two_cell_board,))
        self.assertEqual(replace(build, airframe=airframe).incompatibilities(), ["motor isn't rated for 2S"])

    def test_worst_case_is_worse_on_every_count(self):
        nominal = self.build.evaluate(REQUIREMENTS, TECH)
        worst = UNCERTAINTY.worst_case(self.build, TECH).evaluate(REQUIREMENTS, TECH)
        self.assertLess(worst.flight_time_s, nominal.flight_time_s)
        self.assertLess(worst.thrust_to_weight, nominal.thrust_to_weight)


class SearchTest(unittest.TestCase):
    def setUp(self):
        motors = (MOTOR, replace(MOTOR, name="heavier", mass_kg=3e-3))
        propellers = (PROPELLER, replace(PROPELLER, name="too big", diameter_m=90 * METERS_PER_MILLIMETER))
        batteries = (BATTERY, replace(BATTERY, name="2S", cell_count=2))
        airframes: tuple[Airframe, ...] = (reference.AIRFRAME,)
        self.result = search(REQUIREMENTS, airframes, motors, propellers, batteries, TECH, UNCERTAINTY)

    def test_search_tries_every_combination_and_sets_aside_misfits(self):
        self.assertEqual(self.result.combination_count, 8)
        # Only the small prop on the 1S pack fits, once per motor.
        self.assertEqual(len(self.result.evaluations), 2)
        self.assertEqual(len(self.result.incompatible), 6)
        self.assertTrue(all(math.isfinite(e.nominal.flight_time_s) for e in self.result.evaluations))

    def test_ranking_puts_passing_builds_first_then_the_closest_to_passing(self):
        ranked = self.result.ranked
        self.assertEqual(len(ranked), 2)
        statuses = [evaluation.passes for evaluation in ranked]
        self.assertEqual(statuses, sorted(statuses, reverse=True))
        failing = [evaluation for evaluation in ranked if not evaluation.passes]
        margins = [evaluation.nominal.tightest_check.margin for evaluation in failing]
        self.assertEqual(margins, sorted(margins, reverse=True))


class MotorGapFillingTest(unittest.TestCase):
    def test_listed_numbers_are_kept_and_missing_ones_estimated(self):
        scaling = MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT)
        partial = scaling.motor_with_kv("partial", 15000, 3e-3, resistance_ohm=0.2)
        self.assertEqual(partial.resistance_ohm, 0.2)
        self.assertEqual(partial.estimated_fields, ("no-load current", "max current"))

    def test_estimates_for_a_motor_like_the_reference_reproduce_it(self):
        # Same Kv and mass as the 08028, nothing listed: the estimate is the 08028 itself.
        scaling = MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT)
        estimate = scaling.motor_with_kv("estimate", MOTOR.kv_rpm_per_v, MOTOR.mass_kg)
        self.assertAlmostEqual(estimate.resistance_ohm, MOTOR.resistance_ohm)
        self.assertAlmostEqual(estimate.max_current_a, MOTOR.max_current_a)
        speed_rad_s = 2000.0
        self.assertAlmostEqual(
            estimate.drag_torque_nm(speed_rad_s, SPEED_EXPONENT), MOTOR.drag_torque_nm(speed_rad_s, SPEED_EXPONENT)
        )


class CatalogTest(unittest.TestCase):
    def test_csv_catalog_matches_the_reference_parts(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder / "motors.csv").write_text(
                "name,kv_rpm_per_v,mass_g,max_current_a,max_power_w,max_cells,shaft_mm,price_usd,"
                "resistance_ohm,no_load_current_a,no_load_test_voltage_v\n"
                "ref,10000,2.4,1.8,,1,1.0,,0.52,0.40,4.0\n"
                "partial,15000,3.0,,,,1.5,9.5,,,\n"
                "listing,11500,3.8,,37,2,1.5,,,,\n"
                "#ignored,1,1,,,,,,,,\n"
            )
            (folder / "propellers.csv").write_text(
                "name,diameter_mm,pitch_mm,blades,mass_g,bore_mm,price_usd,test_motor,test_voltage_v,"
                "test_current_a,test_thrust_g\n"
                "prop,55,,,0.33,1.0,,ref,4.0,1.8,30\n"
                "listed only,65,38,2,0.5,1.0,,,,,\n"
            )
            (folder / "batteries.csv").write_text(
                "name,cells,capacity_mah,mass_g,continuous_c,burst_c,chemistry,price_usd\n"
                "pack,1,350,9.1,15,30,,\n"
                "high voltage pack,1,300,7,30,60,LiHV,\n"
            )
            motor_scaling = load_motor_scaling(folder / "motors.csv", TECH, fitted=None)
            motors = load_motors(folder / "motors.csv", TECH, motor_scaling)
            propellers = load_propellers(folder / "propellers.csv", motors, TECH, PROP_SCALING)
            batteries = load_batteries(folder / "batteries.csv")

        self.assertEqual([motor.name for motor in motors], ["ref", "partial", "listing"])
        self.assertEqual(motors[0].estimated_fields, ())
        self.assertEqual(motors[1].estimated_fields, ("resistance", "no-load current", "max current"))

        # Max current from max power at the rated cells: 37 W / (2 x 3.7 V) = 5 A.
        self.assertAlmostEqual(motors[2].max_current_a, 5.0)
        self.assertEqual(motors[2].max_cell_count, 2)
        self.assertIn("max current from max power", motors[2].estimated_fields)

        self.assertAlmostEqual(propellers[0].thrust_coefficient, PROPELLER.thrust_coefficient)
        self.assertAlmostEqual(propellers[0].power_coefficient, PROPELLER.power_coefficient)
        self.assertFalse(propellers[0].estimated)

        # A prop listed only by size and pitch gets the rule's coefficients and is marked estimated.
        self.assertTrue(propellers[1].estimated)
        self.assertAlmostEqual(propellers[1].thrust_coefficient, PROP_SCALING.thrust_coefficient(38 / 65, 0.065))

        self.assertAlmostEqual(batteries[0].capacity_ah, BATTERY.capacity_ah)
        self.assertAlmostEqual(batteries[0].mass_kg, BATTERY.mass_kg)
        # A pack with no chemistry listed is LiPo; a LiHV pack charges higher and holds more energy per Ah.
        self.assertIs(batteries[0].chemistry, LIPO)
        self.assertAlmostEqual(batteries[1].fresh_voltage_v, 4.35)
        self.assertAlmostEqual(batteries[1].energy_wh, 3.8 * 0.3)

    def test_estimated_parts_get_wider_error_bars(self):
        scaling = MotorScaling.from_one_motor(MOTOR, SPEED_EXPONENT)
        estimated_motor = scaling.motor_with_kv("estimated", MOTOR.kv_rpm_per_v, MOTOR.mass_kg)

        listed_worst = UNCERTAINTY.worst_case_motor(MOTOR)
        estimated_worst = UNCERTAINTY.worst_case_motor(estimated_motor)

        # A datasheet motor keeps its resistance; an estimated one gets K_m 30% lower.
        self.assertAlmostEqual(listed_worst.resistance_ohm, MOTOR.resistance_ohm)
        self.assertAlmostEqual(
            estimated_worst.motor_constant_nm_per_sqrt_w, 0.7 * estimated_motor.motor_constant_nm_per_sqrt_w
        )
        # Drag rises 30% for the datasheet motor and 100% for the estimated one, at any speed.
        speed_rad_s = 2000.0
        self.assertAlmostEqual(
            listed_worst.drag_torque_nm(speed_rad_s, SPEED_EXPONENT), 1.3 * MOTOR.drag_torque_nm(speed_rad_s, SPEED_EXPONENT)
        )
        self.assertAlmostEqual(
            estimated_worst.drag_torque_nm(speed_rad_s, SPEED_EXPONENT),
            2.0 * estimated_motor.drag_torque_nm(speed_rad_s, SPEED_EXPONENT),
        )

        listed_prop = UNCERTAINTY.worst_case_propeller(PROPELLER)
        estimated_prop = UNCERTAINTY.worst_case_propeller(replace(PROPELLER, estimated=True))
        self.assertAlmostEqual(listed_prop.thrust_coefficient, 0.9 * PROPELLER.thrust_coefficient)
        self.assertAlmostEqual(estimated_prop.thrust_coefficient, 0.75 * PROPELLER.thrust_coefficient)


if __name__ == "__main__":
    unittest.main()
