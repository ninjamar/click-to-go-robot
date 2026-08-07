// Encoder disk for the wheel odometer.
// Defaults reproduce the original disk measured off the robot: 12 evenly
// spaced blades, 24.30 mm across the tips, on a ~2.00 mm shaft.
//
// Only the values under INPUTS are free. Everything in DERIVED follows from
// them, so changing one input can't leave the model inconsistent.

$fn = 180;

// ---------------------------------------------------------------- INPUTS

/* [Blades] */
blades       = 12;
blade_length = 6.00;   // hub edge to blade tip; grows the disk, not the hub
duty_root    = 0.56;   // fraction of the circular pitch a blade fills at the root
duty_tip     = 0.47;   // ... and at the tip
thickness    = 0.50;

/* [Shaft] */
d_shaft      = 2.15;   // tuned for fit, not the nominal 2.00 shaft
d_mouth      = 3.00;   // funnel mouth on the face opposite the cap

/* [Cap] */
d_cap        = 4.90;
h_cap        = 5.20;   // above the disk face
hub_lip      = 3.70;   // radial web between the cap wall and the blade roots

/* [Print] */
min_gap      = 0.40;   // narrowest printable slot, one nozzle width

// --------------------------------------------------------------- DERIVED

r_hub = d_cap / 2 + hub_lip;
r_tip = r_hub + blade_length;

d_total = 2 * r_tip;
h_total = thickness + h_cap;

// Circular pitch at each end of a blade, and the blade width that fills it
pitch_root = 2 * PI * r_hub / blades;
pitch_tip  = 2 * PI * r_tip / blades;
blade_root_w = duty_root * pitch_root;
blade_tip_w  = duty_tip  * pitch_tip;

gap_root = pitch_root - blade_root_w;
gap_tip  = pitch_tip  - blade_tip_w;

// The funnel spans the disk, so the taper angle falls out of the thickness
h_funnel     = thickness;
funnel_angle = atan((d_mouth - d_shaft) / 2 / h_funnel);  // from vertical

overlap = 0.01;  // sinks the blade root into the hub so the union is clean

// ---------------------------------------------------------------- CHECKS

assert(blades >= 2);
assert(blade_length > 0);
assert(hub_lip > 0, "Hub must be wider than the cap");
assert(d_shaft < d_mouth, "Funnel mouth must be wider than the bore");
assert(d_mouth < d_cap, "Funnel would break out through the cap wall");
assert(duty_root > 0 && duty_root < 1);
assert(duty_tip  > 0 && duty_tip  < 1);
assert(gap_root >= min_gap,
       "Blade roots are too close to print; fewer blades or a bigger hub_lip");

echo(str("d_total = ", d_total, " mm, h_total = ", h_total, " mm"));
echo(str("blade width  root ", blade_root_w, " tip ", blade_tip_w, " mm"));
echo(str("slot width   root ", gap_root, " tip ", gap_tip, " mm"));
echo(str("funnel ", funnel_angle, " deg from vertical"));

// ----------------------------------------------------------------- MODEL

module blade_2d() {
    polygon([
        [r_hub - overlap, -blade_root_w / 2],
        [r_tip,           -blade_tip_w  / 2],
        [r_tip,            blade_tip_w  / 2],
        [r_hub - overlap,  blade_root_w / 2],
    ]);
}

module disk_2d() {
    union() {
        circle(r = r_hub);
        for (i = [0 : blades - 1])
            rotate(i * 360 / blades) blade_2d();
    }
}

module encoder_disk() {
    difference() {
        union() {
            linear_extrude(height = thickness) disk_2d();
            cylinder(h = h_total, d = d_cap);
        }
        // Shaft bore, through the disk and the cap
        translate([0, 0, -1]) cylinder(h = h_total + 2, d = d_shaft);

        // Funnel on the face opposite the cap. The straight section sits
        // below z=0, outside the part, so the mouth is exactly d_mouth.
        cylinder(h = h_funnel, d1 = d_mouth, d2 = d_shaft);
        translate([0, 0, -1]) cylinder(h = 1, d = d_mouth);
    }
}

encoder_disk();
