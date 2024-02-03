#!/usr/bin/env python3

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Pango, Gdk, GLib

class AutonomousModeWindow(Gtk.Window):

    def __init__(self):
        super().__init__(title="Base Station")
        self.set_default_size(200, 75)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.override_background_color(Gtk.StateType.NORMAL, Gdk.RGBA(0, 0, 0, 1))  # Set background color to black

        # Add icon to label and resize it
        icon = Gtk.Image.new_from_file("rocket.png")
        icon.set_pixel_size(50)  # Adjust the pixel size to resize the image
        self.label = Gtk.Label()
        self.label.set_text("Preparing Base Station")
        self.label.set_justify(Gtk.Justification.CENTER)
        self.label.modify_fg(Gtk.StateType.NORMAL, Gdk.color_parse("#00FF00"))  # Set text color to green
        self.label.modify_font(Pango.FontDescription("Bold 16"))  # Set text to bold and size 16

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        box.pack_start(self.label, True, True, 0)
        box.pack_start(icon, True, True, 0)

        self.add(box)

        self.progressbar = Gtk.ProgressBar()
        self.progressbar.set_fraction(0.0)  # Initialize progress to 0%
        self.progressbar.set_pulse_step(0.1)  # Set pulse step for animation
        self.progressbar.set_size_request(-1, 20)  # Set the height of the progress bar

        box.pack_start(self.progressbar, True, True, 0)

        # Start the pulse animation
        self.timeout_id = GLib.timeout_add(100, self.on_timeout)

        # Simulate progress completion after 5 seconds
        GLib.timeout_add_seconds(2, self.on_progress_complete)

    def on_timeout(self):
        self.progressbar.pulse()
        return True

    def on_progress_complete(self):
        # Stop the pulse animation
        GLib.source_remove(self.timeout_id)

        # Set progress to 100%
        self.progressbar.set_fraction(1.0)

        self.label.set_text("Ready for Launch")
        GLib.timeout_add_seconds(2, self.close_window)  # Close the window after 2 seconds
        return False

    def close_window(self):
        self.destroy()

win = AutonomousModeWindow()
win.connect("destroy", Gtk.main_quit)
win.show_all()
Gtk.main()
