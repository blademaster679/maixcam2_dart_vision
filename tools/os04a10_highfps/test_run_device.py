"""Host-only launcher-command checks; no device or real processes are touched."""
import shlex
import subprocess
import unittest
from types import SimpleNamespace

from run_device import development_enter_command


class HeadlessEntryTests(unittest.TestCase):
    def setUp(self):
        self.launcher = SimpleNamespace(
            enter_command=lambda: "echo graphical_entry",
            kill_launcher_daemon_command=lambda: "echo daemon_stopped",
            ensure_maixapp_apps_stopped_command=lambda: "echo apps_stop_checked",
        )

    def run_entry(self, processes, pause_camera=False):
        command = development_enter_command(self.launcher, True, pause_camera)
        # The generated guard runs genuine awk against synthetic ps output.
        fake_ps = "ps() { printf '%s\\n' " + shlex.quote(processes) + "; }; "
        return subprocess.run(["sh", "-c", fake_ps + command],
                              text=True, capture_output=True, timeout=5)

    def test_default_keeps_graphical_entry(self):
        self.assertEqual(development_enter_command(self.launcher), "echo graphical_entry")

    def test_headless_never_calls_graphical_helper(self):
        def reject_display():
            raise AssertionError("headless entry initialized a display")
        self.launcher.enter_command = reject_display
        result = self.run_entry("/maixapp/apps/launcher/launcher\n/usr/sbin/sshd")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("apps_stop_checked", result.stdout)
        self.assertIn("development_mode=entered_headless", result.stdout)

    def test_unrelated_app_is_not_stopped(self):
        for process in ("/maixapp/apps/other/main --option",
                        "python3 /maixapp/apps/other/main.py",
                        "/bin/sh /maixapp/apps/other/main.sh"):
            with self.subTest(process=process):
                result = self.run_entry(process)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("apps_stop_checked", result.stdout)
                self.assertIn("Another app is active", result.stderr)

    def test_camera_requires_explicit_pause_and_does_not_exempt_other_apps(self):
        camera = "/maixapp/apps/camera/camera"
        self.assertNotEqual(self.run_entry(camera).returncode, 0)
        self.assertEqual(self.run_entry(camera, pause_camera=True).returncode, 0)
        result = self.run_entry(camera + "\n/maixapp/apps/other/main", pause_camera=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("apps_stop_checked", result.stdout)


if __name__ == "__main__":
    unittest.main()
