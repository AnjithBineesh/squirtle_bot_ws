"""Unified CLI and interactive launcher for Squirtle Bot."""

from __future__ import annotations

import argparse
import sys
import unittest

from squirtle_bot.tests.motor_test import run_motor_test


def run_unit_tests() -> None:
    """Run the test suite."""
    print("=" * 60)
    print("Running Squirtle Bot Automated Unit Test Suite...")
    print("=" * 60)
    suite = unittest.defaultTestLoader.discover("squirtle_bot.tests", pattern="test_*.py")
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    if not result.wasSuccessful():
        sys.exit(1)


def show_menu() -> None:
    """Display interactive menu to choose any robot subsystem test."""
    while True:
        print("\n" + "=" * 62)
        print(" SQUIRTLE BOT — ROBOT CONTROL & DIAGNOSTIC MENU")
        print("=" * 62)
        print(" 1. Interactive Motor Test (F/B/L/R/S/Q)")
        print(" 2. Live Sensor Diagnostic (IMU + Encoders live terminal)")
        print(" 3. Encoder Manual Test (Wheel tick counting)")
        print(" 4. Encoder Calibration (Measure ticks per wheel revolution)")
        print(" 5. Odometry Line & Wheel Speed Sync Test")
        print(" 6. Calibrated Distance Drive Test")
        print(" 7. Calibrated Turn Angle Test")
        print(" 8. 4-Corner Square Path Test")
        print(" 9. Autonomous Waypoint Navigation Test")
        print("10. Run Automated Unit Tests")
        print("11. Start Network Robot Server (Listen for Laptop commands)")
        print(" Q. Quit")
        print("=" * 62)

        try:
            choice = input("Select an option (1-11 or Q): ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            break

        if choice in {"q", "quit", "exit"}:
            print("Exiting Squirtle Bot launcher.")
            break
        elif choice == "1":
            run_motor_test()
        elif choice == "2":
            from squirtle_bot.tests.sensor_diagnostic import run_diagnostic
            run_diagnostic(duration_seconds=30.0)
        elif choice == "3":
            from squirtle_bot.tests.encoder_test import main as enc_main
            enc_main()
        elif choice == "4":
            from squirtle_bot.tests.encoder_calibration import main as cal_main
            cal_main()
        elif choice == "5":
            from squirtle_bot.tests.odometry_line_test import main as odom_main
            odom_main()
        elif choice == "6":
            from squirtle_bot.tests.distance_drive_test import main as dist_main
            dist_main()
        elif choice == "7":
            from squirtle_bot.tests.turn_angle_test import main as turn_main
            turn_main()
        elif choice == "8":
            from squirtle_bot.tests.square_path_test import main as square_main
            square_main()
        elif choice == "9":
            from squirtle_bot.tests.waypoint_navigation_test import main as wp_main
            wp_main()
        elif choice == "10":
            run_unit_tests()
        elif choice == "11":
            from squirtle_bot.hardware.robot_server import run_server
            run_server()
        else:
            print(f"Invalid option: '{choice}'. Please select from 1-11 or Q.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Squirtle Bot — Main Controller and Subsystem Launcher",
        add_help=True,
    )
    parser.add_argument(
        "subcommand",
        nargs="?",
        default=None,
        choices=[
            "motor",
            "menu",
            "diagnostic",
            "encoders",
            "calibrate",
            "odometry",
            "distance",
            "turn",
            "square",
            "waypoint",
            "vision",
            "server",
            "test",
        ],
        help="Specific test/controller to run directly",
    )
    parser.add_argument(
        "--motor",
        action="store_true",
        help="Run interactive low-speed motor test (default for headless/piped input)",
    )
    parser.add_argument(
        "--menu",
        action="store_true",
        help="Open interactive menu for all tests and diagnostics",
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run automated unit test suite",
    )

    args = parser.parse_args()

    # Route based on explicit arguments or subcommands
    if args.test or args.subcommand == "test":
        run_unit_tests()
        return

    if args.motor or args.subcommand == "motor":
        run_motor_test()
        return

    if args.subcommand == "diagnostic":
        from squirtle_bot.tests.sensor_diagnostic import run_diagnostic
        run_diagnostic(duration_seconds=30.0)
        return

    if args.subcommand == "encoders":
        from squirtle_bot.tests.encoder_test import main as enc_main
        enc_main()
        return

    if args.subcommand == "calibrate":
        from squirtle_bot.tests.encoder_calibration import main as cal_main
        cal_main()
        return

    if args.subcommand == "odometry":
        from squirtle_bot.tests.odometry_line_test import main as odom_main
        odom_main()
        return

    if args.subcommand == "distance":
        from squirtle_bot.tests.distance_drive_test import main as dist_main
        dist_main()
        return

    if args.subcommand == "turn":
        from squirtle_bot.tests.turn_angle_test import main as turn_main
        turn_main()
        return

    if args.subcommand == "square":
        from squirtle_bot.tests.square_path_test import main as square_main
        square_main()
        return

    if args.subcommand == "waypoint":
        from squirtle_bot.tests.waypoint_navigation_test import main as wp_main
        wp_main()
        return

    if args.subcommand == "server":
        from squirtle_bot.hardware.robot_server import run_server
        run_server()
        return

    # If --menu or interactive terminal without subcommand:
    if args.menu or args.subcommand == "menu":
        show_menu()
        return

    # Default behaviour: if connected to a TTY, show menu; otherwise run motor test for backward compatibility
    if sys.stdin.isatty():
        show_menu()
    else:
        run_motor_test()


if __name__ == "__main__":
    main()
